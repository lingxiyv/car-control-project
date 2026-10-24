//
// Created by lingxi on 2026/10/14.
//

#include "ChassisController.h"

#include <cmath>

#include "Angle.hpp"
#include "stm32f4xx_hal.h"

namespace {
    //角度归一化与钳位是纯数学工具，已下沉到 core/math/Angle.hpp
    using core_math::clampFloat;
    using core_math::kPi;
    using core_math::normalizeAngle;

    constexpr float R = ChassisParam::WHEEL_RADIUS;
    constexpr float K = (ChassisParam::WHEEL_BASE + ChassisParam::TRACK_W) / 2.0f;
    constexpr float V_TO_RPS = 1.0f / (2.0f * kPi * R);  // 线速度 → 轮轴转速
    constexpr float RPS_TO_V = 1.0f / V_TO_RPS;          // 轮轴转速 → 线速度

    /**
     * @brief 世界系速度 -> 车体系速度
     * @note  v_body = R(-yaw) * v_world，其中 R 为绕 z 轴的旋转矩阵
     *        omega 是绕 z 轴的角速度，平面运动下两个坐标系里数值相同，原样带过去
     */
    ChassisVelocity worldToBody(const ChassisVelocity& v_world, float yaw) {
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        return ChassisVelocity(v_world.vx * c + v_world.vy * s,
                               -v_world.vx * s + v_world.vy * c,
                               v_world.omega);
    }

    /**
     * @brief 车体系速度 -> 世界系速度
     * @note  v_world = R(yaw) * v_body
     */
    ChassisVelocity bodyToWorld(const ChassisVelocity& v_body, float yaw) {
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        return ChassisVelocity(v_body.vx * c - v_body.vy * s,
                               v_body.vx * s + v_body.vy * c,
                               v_body.omega);
    }
}

ChassisController::ChassisController(IWheelDriver& wheel_driver, IHeadingSensor& heading_sensor) :
    wheel_driver_(wheel_driver), heading_sensor_(heading_sensor) {
}

bool ChassisController::init() {
    //初始化底层硬件
    if (!wheel_driver_.init() || !heading_sensor_.init()) {
        return false;
    }

    //四路速度环 PID
    for (uint8_t i = 0; i < kWheelCount; i++) {
        wheel_pid_[i].setOutputLimit(-1.0f, 1.0f);
        wheel_pid_[i].setIntegralLimit(1.5f);
        wheel_pid_[i].reset();
    }

    //角速度环默认增益(占位值，需实车标定)
    course_kp_ = ChassisParam::COURSE_KP;
    course_ki_ = ChassisParam::COURSE_KI;
    course_kd_ = ChassisParam::COURSE_KD;
    spin_kp_   = ChassisParam::SPIN_KP;
    spin_ki_   = ChassisParam::SPIN_KI;
    spin_kd_   = ChassisParam::SPIN_KD;

    //各类上限
    max_wheel_speed_      = MotorParam::MAX_WHEEL_SPEED;
    max_translate_speed_  = ChassisParam::MAX_TRANSLATE_SPEED;
    max_spin_rate_        = ChassisParam::MAX_SPIN_RATE;
    max_omega_correction_ = ChassisParam::MAX_OMEGA_CORRECTION;

    //世界系基准：这里刚做过 resetYaw()，此刻车头方向即世界系 +x，所以基准取 0。
    //(不取 0 的话，update() 里算 relative_yaw 会用到未初始化的成员)
    heading_sensor_.resetYaw();
    world_coordinate_system_reference_course = 0.0f;

    //运动目标全部清零
    velocity_target_world_  = ChassisVelocity();
    velocity_current_       = ChassisVelocity();
    velocity_current_world_ = ChassisVelocity();
    course_target_rad_      = 0.0f;
    spin_target_radps_        = 0.0f;

    //角速度环回到 kIdle
    angular_control_ = AngularControl::kIdle;
    angular_pid_.reset();
    applyAngularGains(AngularControl::kIdle);

    //标志与状态复位
    data_valid_ = false;
    enabled_    = false;
    brake_mode_ = BrakeMode::k_;
    return true;
}

bool ChassisController::calibrationIMU() {
    if (!heading_sensor_.isInitialized()) return false;
    heading_sensor_.resetYaw();

    float sum = 0.0f;
    for (int i = 0; i < 2000; i++) {
        sum += heading_sensor_.getYawRate();
        HAL_Delay(1);
    }
    heading_sensor_.setStaticOffset(sum / 2000.0f);

    //把标定结束时刻的车头方向定义为世界系 +x
    world_coordinate_system_reference_course = heading_sensor_.getYaw();
    //世界系变了，把航迹目标对齐到新的 +x，避免标定完车自己转一下
    course_target_rad_ = 0.0f;
    return true;
}

bool ChassisController::isReady() const {
    return wheel_driver_.isInitialized() && heading_sensor_.isInitialized()
           && heading_sensor_.isCalibrated();
}

void ChassisController::setEnabled(bool enabled) {
    if (enabled) {
        enabled_ = true;
        wheel_driver_.setEnabled(true);
        //清空PID内部状态
        for (uint8_t i = 0; i < kWheelCount; i++) {
            wheel_pid_[i].reset();
        }
        angular_pid_.reset();
        brake_mode_ = BrakeMode::k_;
    } else {
        enabled_ = false;
        wheel_driver_.setEnabled(false);
        //失能时清掉残留的刹车状态：否则 stop() 还会继续对一个已失能的驱动
        //做"恢复PWM"之类的动作，状态与硬件对不上
        brake_mode_ = BrakeMode::k_;
    }
}

bool ChassisController::isEnabled() const {
    return enabled_;
}

//==============================================================================
// 运动目标下达
//==============================================================================

bool ChassisController::setCourseTarget(float course_rad, float speed_mps) {
    if (!isReady()) {
        return false;
    }

    //速度大小：负值按 0 处理(要反向请把 course_rad 加 π)，超限按上限
    const float speed  = clampFloat(speed_mps, 0.0f, max_translate_speed_);
    const float course = normalizeAngle(course_rad);

    //分解到世界系 (vx, vy)
    velocity_target_world_.vx    = speed * std::cos(course);
    velocity_target_world_.vy    = speed * std::sin(course);
    velocity_target_world_.omega = 0.0f;   //角速度由角速度环独占，不在这里

    course_target_rad_ = course;
    //被顶掉的自旋指令要真正清零，否则 getSpinTarget() 会一直返回一个
    //已经不起作用的旧目标，看起来像"还在自旋"
    spin_target_radps_ = 0.0f;

    //夺取角速度环控制权(与 setSpinTarget() 互斥)
    switchAngularControl(AngularControl::kCourse);
    requestResumeFromBrake();
    return true;
}

bool ChassisController::setSpinTarget(float omega_radps) {
    if (!isReady()) {
        return false;
    }

    spin_target_radps_ = clampFloat(omega_radps, -max_spin_rate_, max_spin_rate_);

    //夺取角速度环控制权(与 setCourseTarget() 互斥)；平移目标保持不变。
    //course_target_rad_ 这里不清零：它只是"挂着但不生效"，
    //下一次 setCourseTarget() 必然会写入新值
    switchAngularControl(AngularControl::kSpin);
    requestResumeFromBrake();
    return true;
}

void ChassisController::releaseAngularControl() {
    //把航迹目标对齐到当前朝向，之后若重新进入 kCourse 不会突转
    if (isReady()) {
        course_target_rad_ = currentYawWorld();
    } else {
        course_target_rad_ = 0.0f;
    }
    spin_target_radps_ = 0.0f;
    switchAngularControl(AngularControl::kIdle);
}

AngularControl ChassisController::getAngularControl() const {
    return angular_control_;
}

void ChassisController::setWorldTranslateTarget(float vx_world, float vy_world) {
    //只改平移目标，不夺取也不归还角速度环控制权
    velocity_target_world_.vx = vx_world;
    velocity_target_world_.vy = vy_world;
    requestResumeFromBrake();
}

//==============================================================================
// 状态读取
//==============================================================================

ChassisVelocity ChassisController::getTranslateTargetWorld() const {
    return ChassisVelocity(velocity_target_world_.vx, velocity_target_world_.vy, 0.0f);
}

float ChassisController::getCourseTarget() const {
    return course_target_rad_;
}

float ChassisController::getSpinTarget() const {
    return spin_target_radps_;
}

ChassisVelocity ChassisController::getCurrentVelocity() const {
    //读端才是真正需要临界区的地方：写入发生在 update()(中断上下文)，
    //这里保存并恢复 PRIMASK，而不是无条件开中断
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const ChassisVelocity snapshot = velocity_current_;
    __set_PRIMASK(primask);
    return snapshot;
}

ChassisVelocity ChassisController::getCurrentVelocityWorld() const {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const ChassisVelocity snapshot = velocity_current_world_;
    __set_PRIMASK(primask);
    return snapshot;
}

bool ChassisController::isDataValid() const {
    return data_valid_;
}

//==============================================================================
// 参数配置
//==============================================================================

void ChassisController::setWheelPidParameters(float kp, float ki, float kd) {
    for (uint8_t i = 0; i < kWheelCount; i++) {
        wheel_pid_[i].setParameters(kp, ki, kd);
    }
}

void ChassisController::setCoursePidParameters(float kp, float ki, float kd) {
    course_kp_ = kp;
    course_ki_ = ki;
    course_kd_ = kd;
    if (angular_control_ == AngularControl::kCourse) {
        angular_pid_.reset();   //换增益就清积分，避免带旧参数累积的量突变
        applyAngularGains(AngularControl::kCourse);
    }
}

void ChassisController::setSpinPidParameters(float kp, float ki, float kd) {
    spin_kp_ = kp;
    spin_ki_ = ki;
    spin_kd_ = kd;
    if (angular_control_ == AngularControl::kSpin) {
        angular_pid_.reset();
        applyAngularGains(AngularControl::kSpin);
    }
}

void ChassisController::setMaxWheelSpeed(float rps) {
    if (rps <= 0.0f) return;    //非正值会让限幅比例失去意义
    max_wheel_speed_ = rps;
}

float ChassisController::getMaxWheelSpeed() const {
    return max_wheel_speed_;
}

void ChassisController::setMaxTranslateSpeed(float mps) {
    if (mps <= 0.0f) return;
    max_translate_speed_ = mps;
}

float ChassisController::getMaxTranslateSpeed() const {
    return max_translate_speed_;
}

void ChassisController::setMaxSpinRate(float radps) {
    if (radps <= 0.0f) return;
    max_spin_rate_ = radps;
    //目标与输出限幅都要跟着变
    spin_target_radps_ = clampFloat(spin_target_radps_, -max_spin_rate_, max_spin_rate_);
    if (angular_control_ == AngularControl::kSpin) {
        applyAngularGains(AngularControl::kSpin);
    }
}

float ChassisController::getMaxSpinRate() const {
    return max_spin_rate_;
}

//==============================================================================
// 停机
//==============================================================================

void ChassisController::normalstop() {
    velocity_target_world_ = ChassisVelocity();
    releaseAngularControl();
    //已经在刹车流程里就不要重开：重开会把 brake_mode_ 退回 kCoast，
    //stop() 于是再次进入 kCoast 分支并**重新臂** brake_tick_，
    //超时永远触发不了 —— 表现为持续按着"停止"时状态机一直停在 kCoasting
    if (brake_mode_ == BrakeMode::k_) {
        brake_mode_ = BrakeMode::kCoast;
    }
}

void ChassisController::emergencyStop() {
    velocity_target_world_ = ChassisVelocity();
    spin_target_radps_ = 0.0f;
    if (isReady()) {
        course_target_rad_ = currentYawWorld();
    }
    switchAngularControl(AngularControl::kIdle);

    //先失能：它会落到滑行态并把 brake_mode_ 清成 k_
    setEnabled(false);
    //再**同步**上急刹，不要把刹车推迟到下一次 stop()。
    //否则调用方只要没有周期性地泵 stop()，车就是纯滑行的，急刹等于没做
    wheel_driver_.BrakeStop();
    brake_tick_ = HAL_GetTick();
    brake_mode_ = BrakeMode::kBraking;   // 直接进入"刹车执行中"，不经过 kBrake

    for (uint8_t i = 0; i < kWheelCount; i++) {
        wheel_pid_[i].reset();
    }
    angular_pid_.reset();
}

void ChassisController::stop(float dt) {
    //这里**不能**用 isReady()：它要求 IMU 已校准，而刹车只依赖轮速反馈。
    //用 isReady() 当门禁会导致"IMU 没标定时连停都停不下来"
    if (!wheel_driver_.isInitialized() || brake_mode_ == BrakeMode::k_) {
        return;
    }

    //刹车流程期间自己刷新速度反馈(车体系与世界系两份快照同时写)。
    //stop() 与 update() 是互斥入口，不能指望 update() 已经在跑；
    //不刷新的话 velocity_current_ 会冻结在进刹车前那一拍，永远判不出"已停"。
    //
    //dt 非法时**仍然要推进状态机**，只是跳过刷新：
    //1 kHz 控制周期下用 HAL_GetTick() 算出来的 dt 会周期性出现 0，
    //若在这里直接 return，状态机会被永久冻结
    if (dt > 0.0f) {
        const ChassisVelocity body = forwardKinematics(wheel_driver_.getWheelSpeed(dt));
        //世界系快照需要 yaw；IMU 不可用时(比如没标定)就只更新车体系，
        //刹车本身不依赖 IMU，不能因此而不停机
        const bool imu_ok = heading_sensor_.isInitialized() && heading_sensor_.isCalibrated();
        const float yaw = imu_ok ? currentYawWorld() : 0.0f;

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        velocity_current_ = body;
        if (imu_ok) {
            velocity_current_world_ = bodyToWorld(body, yaw);
        }
        data_valid_ = true;
        __set_PRIMASK(primask);
    }

    switch (brake_mode_) {
        case BrakeMode::k_: {
            return;
        }
        case BrakeMode::kTermination: {
            //终止刹车：恢复 PWM 通路后回到正常运行态。
            //这里必须有 break：否则会跌进下面的 kCoast，
            //变成"刚恢复 PWM 又立刻滑行停机"，自相矛盾
            if (enabled_) {
                wheel_driver_.RestorePWM_GPIO();
            }
            brake_mode_ = BrakeMode::k_;
            break;
        }
        case BrakeMode::kCoast: {
            wheel_driver_.CoastStop_WithStartupDelay_PWMDisabled();
            brake_tick_ = HAL_GetTick();
            brake_mode_ = BrakeMode::kCoasting;
            break;
        }
        case BrakeMode::kBrake: {
            wheel_driver_.BrakeStop();
            brake_tick_ = HAL_GetTick();
            brake_mode_ = BrakeMode::kBraking;
            break;
        }
        case BrakeMode::kCoasting:
        case BrakeMode::kBraking: {
            const uint32_t elapsed = HAL_GetTick() - brake_tick_;
            //测速窗口有零阶保持，刚进刹车态的头几拍可能还是 0，
            //所以先等过最小驻留时间，再允许按"已停"退出，避免高速时被误判
            const bool settled  = elapsed >= ChassisParam::STOP_MIN_DWELL_MS;
            const bool stopped  = settled && isChassisStopped();
            const bool timeout  = elapsed >= ChassisParam::STOP_TIMEOUT_MS;
            if (stopped || timeout) {
                //失能状态下不要恢复 PWM 通路：驱动已经被停在滑行/刹车电平上，
                //恢复 PWM 没有意义，这一步交给 setEnabled(true) 去做
                if (enabled_) {
                    wheel_driver_.RestorePWM_GPIO();
                }
                brake_mode_ = BrakeMode::k_;
            }
            break;
        }
        default: break;
    }
}

bool ChassisController::isChassisStopped() const {
    return std::fabs(velocity_current_.vx) < ChassisParam::STOP_LINEAR_EPS &&
           std::fabs(velocity_current_.vy) < ChassisParam::STOP_LINEAR_EPS &&
           std::fabs(velocity_current_.omega) < ChassisParam::STOP_ANGULAR_EPS;
}

//==============================================================================
// 主控制
//==============================================================================

void ChassisController::update(float dt) {
    // 前置检查
    if (!isReady() || dt <= 0.0f) {
        return;
    }

    const float yaw = currentYawWorld();

    //--------------------------------------------------------------------------
    // 1) 角速度通道：按控制权归属**择一**取指令
    //    注意是赋值(omega_cmd = ...)，不是把两路相加 —— 这是本次改造的核心
    //--------------------------------------------------------------------------
    float omega_cmd = 0.0f;
    if (enabled_) {
        switch (angular_control_) {
            case AngularControl::kCourse: {
                //以归一化后的最短路径误差作为 setpoint、feedback 传 0，
                //让积分与微分都作用在"绕最近的一侧转过去"的误差上。
                //注意 kd 默认给 0：误差从 +π 跳到 -π 时微分项会有一次尖峰，
                //需要阻尼的话用 -kd_damp * getYawRate() 单独加更稳
                //
                //另外：全轮等比限幅发生在角速度环**之后**，修正量被比例压小时
                //角速度环自己是看不到的，积分可能继续累积；这里靠
                //COURSE_INTEGRAL_LIMIT 兜住上界，不要把它设得太大
                const float heading_error = normalizeAngle(course_target_rad_ - yaw);
                omega_cmd = angular_pid_.calculate(heading_error, 0.0f, dt);
                break;
            }
            case AngularControl::kSpin: {
                //输入是角速度误差(rad/s)，反馈取陀螺 z 轴角速度；
                //零偏没标定的话积分项会持续累积，导致自旋速度慢慢漂
                omega_cmd = angular_pid_.calculate(spin_target_radps_,
                                                   heading_sensor_.getYawRate(), dt);
                break;
            }
            case AngularControl::kIdle:
            default: {
                omega_cmd = 0.0f;
                break;
            }
        }
        //输出限幅已在 applyAngularGains() 里通过 setOutputLimit() 设好
    }

    //--------------------------------------------------------------------------
    // 2) 平移通道：世界系 -> 车体系
    //    必须转。麦轮逆运动学只在车体系下成立，直接把世界系 (vx, vy) 喂进去，
    //    车体转过 90° 之后仍旧会沿"车头方向"走，世界系航迹就是假的
    //--------------------------------------------------------------------------
    const ChassisVelocity target_body = worldToBody(velocity_target_world_, yaw);
    const ChassisVelocity target_vel(target_body.vx, target_body.vy, omega_cmd);

    // 3) 逆运动学 + 全轮等比限幅
    WheelSpeed target_rps = inverseKinematics(target_vel);
    target_rps = limitWheelSpeed(target_rps);

    // 4) 读取四轮实际转速
    const WheelSpeed actual_rps = wheel_driver_.getWheelSpeed(dt);

    // 5) 四路速度环 PID -> PWM
    WheelPwm pwm;   // 默认全 0
    if (enabled_) {
        pwm.lf = PID_CorrectionToPwm(
            wheel_pid_[kWheelLf].calculate(target_rps.lf, actual_rps.lf, dt));
        pwm.rf = PID_CorrectionToPwm(
            wheel_pid_[kWheelRf].calculate(target_rps.rf, actual_rps.rf, dt));
        pwm.lr = PID_CorrectionToPwm(
            wheel_pid_[kWheelLr].calculate(target_rps.lr, actual_rps.lr, dt));
        pwm.rr = PID_CorrectionToPwm(
            wheel_pid_[kWheelRr].calculate(target_rps.rr, actual_rps.rr, dt));
    }
    // 未使能时 pwm 保持全 0；驱动侧也会用 pwm_output_ready_ 再挡一层
    wheel_driver_.setPwm(pwm);

    // 6) 正运动学：车体系与世界系两份快照同时写入
    const ChassisVelocity velocity_body  = forwardKinematics(actual_rps);
    const ChassisVelocity velocity_world = bodyToWorld(velocity_body, yaw);
    //写端也进临界区：update() 允许在中断里跑，也允许在固定周期的任务里跑，
    //后者情况下一个更高优先级的读任务可能读到只写了一半的 ChassisVelocity。
    //1 kHz 下关中断十几个周期可以忽略
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    velocity_current_       = velocity_body;
    velocity_current_world_ = velocity_world;
    data_valid_             = true;
    __set_PRIMASK(primask);
}

//==============================================================================
// 运动学
//==============================================================================

WheelSpeed ChassisController::inverseKinematics(const ChassisVelocity& vel) const {
    WheelSpeed wheel_speed;
    wheel_speed.lf = (vel.vx - vel.vy - vel.omega * K) * V_TO_RPS;
    wheel_speed.rf = (vel.vx + vel.vy + vel.omega * K) * V_TO_RPS;
    wheel_speed.lr = (vel.vx + vel.vy - vel.omega * K) * V_TO_RPS;
    wheel_speed.rr = (vel.vx - vel.vy + vel.omega * K) * V_TO_RPS;
    return wheel_speed;
}

WheelSpeed ChassisController::limitWheelSpeed(const WheelSpeed& cmd) const {
    //取4轮的最大轮速
    const float m = std::fmax(std::fmax(std::fabs(cmd.lf), std::fabs(cmd.rf)),
                              std::fmax(std::fabs(cmd.lr), std::fabs(cmd.rr)));
    //未超限，返回原始值
    if (m <= max_wheel_speed_) return cmd;
    //超限，计算限幅比例
    const float ratio = max_wheel_speed_ / m;
    //返回限幅后结果
    return WheelSpeed(cmd.lf * ratio, cmd.rf * ratio, cmd.lr * ratio, cmd.rr * ratio);
}

ChassisVelocity ChassisController::forwardKinematics(const WheelSpeed& wheel_speeds) const {
    ChassisVelocity velocity;
    velocity.vx = (wheel_speeds.lf + wheel_speeds.rf + wheel_speeds.lr + wheel_speeds.rr)
                  * RPS_TO_V / 4.0f;
    velocity.vy = (-wheel_speeds.lf + wheel_speeds.rf + wheel_speeds.lr - wheel_speeds.rr)
                  * RPS_TO_V / 4.0f;
    velocity.omega = (-wheel_speeds.lf + wheel_speeds.rf - wheel_speeds.lr + wheel_speeds.rr)
                     * RPS_TO_V / (4.0f * K);
    return velocity;
}

int16_t ChassisController::PID_CorrectionToPwm(float pidCorrection) {
    int16_t pwm_correction = static_cast<int16_t>(pidCorrection * MotorParam::PWM_MAX);
    if (pwm_correction > MotorParam::PWM_MAX) {
        pwm_correction = MotorParam::PWM_MAX;
    } else if (pwm_correction < -MotorParam::PWM_MAX) {
        pwm_correction = -MotorParam::PWM_MAX;
    }
    return pwm_correction;
}

//==============================================================================
// 角速度环控制权
//==============================================================================

void ChassisController::switchAngularControl(AngularControl next) {
    if (next == angular_control_) {
        //同模式重复下发是空操作：上层可以在控制周期里持续调用 setCourseTarget()，
        //若这里每次都 reset()，积分量永远攒不起来，闭环会被悄悄废掉
        return;
    }
    angular_control_ = next;
    angular_pid_.reset();          //换源必须清状态，否则旧积分立刻冲击
    applyAngularGains(next);
}

void ChassisController::applyAngularGains(AngularControl mode) {
    switch (mode) {
        case AngularControl::kCourse: {
            angular_pid_.setParameters(course_kp_, course_ki_, course_kd_);
            angular_pid_.setOutputLimit(-max_omega_correction_, max_omega_correction_);
            angular_pid_.setIntegralLimit(ChassisParam::COURSE_INTEGRAL_LIMIT);
            break;
        }
        case AngularControl::kSpin: {
            angular_pid_.setParameters(spin_kp_, spin_ki_, spin_kd_);
            angular_pid_.setOutputLimit(-max_spin_rate_, max_spin_rate_);
            angular_pid_.setIntegralLimit(ChassisParam::SPIN_INTEGRAL_LIMIT);
            break;
        }
        case AngularControl::kIdle:
        default: {
            angular_pid_.setParameters(0.0f, 0.0f, 0.0f);
            angular_pid_.setOutputLimit(-1.0f, 1.0f);
            angular_pid_.setIntegralLimit(ChassisParam::SPIN_INTEGRAL_LIMIT);
            break;
        }
    }
}

float ChassisController::currentYawWorld() {
    return normalizeAngle(heading_sensor_.getYaw() - world_coordinate_system_reference_course);
}

void ChassisController::requestResumeFromBrake() {
    switch (brake_mode_) {
        case BrakeMode::kCoast:
        case BrakeMode::kBrake:
        case BrakeMode::kCoasting:
        case BrakeMode::kBraking: {
            brake_mode_ = BrakeMode::kTermination;
            break;
        }
        default: {
            //本来就没在刹车，什么都不做。
            //这里不能无条件写 kTermination：上层每个周期都下发指令的话，
            //刹车状态会被反复改写，stop() 就会每周期"恢复PWM->立刻停机"
            break;
        }
    }
}
