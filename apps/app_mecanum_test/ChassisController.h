//
// Created by lingxi on 2026/10/14.
//

#ifndef A_TEST_CHASSISCONTROLLER_H
#define A_TEST_CHASSISCONTROLLER_H

#include "IWheelDriver.hpp"
#include "IHeadingSensor.hpp"
#include "PID.h"


/**
 * @brief 停机状态机的状态
 * @note  这是控制层的内部状态，与硬件无关，所以放在 app 层而不是 core/interfaces
 */
enum class BrakeMode {
    k_,          /**< 未触发刹车 */
    kTermination,/**< 终止刹车动作，回到正常运行态 */
    kCoast,      /**< 请求自由滑行 */
    kBrake,      /**< 请求主动刹车(绕组短接) */
    kCoasting,   /**< 滑行动作执行中 */
    kBraking     /**< 主动刹车动作执行中 */
};


/**
 * @brief 底盘机构参数
 * @note  只放机构/控制器层参数；编码器线数与减速比属于驱动层，见 MecanumParam
 */
namespace ChassisParam {
    /** @brief 麦克纳姆轮半径 @unit m */
    constexpr float WHEEL_RADIUS = 0.065f;

    /** @brief 左右轮距 @unit m */
    constexpr float TRACK_W = 0.22f;

    /** @brief 前后轴距 @unit m */
    constexpr float WHEEL_BASE = 0.20f;

    //--------------------------------------------------------------------------
    // 以下为控制层限幅与角速度环默认增益。
    // 全部是占位值：机构装好后必须按实车重新标定，不要直接拿去跑真车。
    //--------------------------------------------------------------------------

    /**
     * @brief 平移速度上限
     * @unit m/s
     * @note  setCourseTarget() 传入的 speed_mps 会被钳到 [0, 该值]
     */
    constexpr float MAX_TRANSLATE_SPEED = 2.0f;

    /**
     * @brief 自旋角速度上限
     * @unit rad/s
     * @note  setSpinTarget() 的目标与角速度环输出都会被钳到 ±该值
     */
    constexpr float MAX_SPIN_RATE = 3.0f;

    /**
     * @brief 航迹模式下角速度环输出的修正量上限
     * @unit rad/s
     * @note  与 MAX_SPIN_RATE 分开：自旋是"允许转多快"，这里是"允许修多猛"
     */
    constexpr float MAX_OMEGA_CORRECTION = 2.0f;

    /**
     * @brief 航迹模式的角速度环默认增益
     * @note  该模式的 PID 输入是**角度误差(rad)**，输出是角速度(rad/s)，
     *        所以 kp 的量纲是 1/s，与自旋模式不可通用
     * @note  kd 默认给 0：航迹误差过 ±π 时微分项会有一次尖峰(见 update() 注释)
     */
    constexpr float COURSE_KP = 2.0f;
    constexpr float COURSE_KI = 0.0f;
    constexpr float COURSE_KD = 0.0f;

    /**
     * @brief 自旋模式的角速度环默认增益
     * @note  该模式的 PID 输入是**角速度误差(rad/s)**，输出也是角速度(rad/s)
     */
    constexpr float SPIN_KP = 2.0f;
    constexpr float SPIN_KI = 0.5f;
    constexpr float SPIN_KD = 0.0f;

    /**
     * @brief 航迹模式的积分项限幅
     * @note  该模式的积分量量纲是 rad·s(角度误差对时间积分)
     */
    constexpr float COURSE_INTEGRAL_LIMIT = 0.5f;

    /**
     * @brief 自旋模式的积分项限幅
     * @note  该模式的积分量量纲是 (rad/s)·s = rad。
     *        陀螺零偏没标定时积分会持续累积，必须给它一个上限
     */
    constexpr float SPIN_INTEGRAL_LIMIT = 2.0f;

    //--------------------------------------------------------------------------
    // 停机状态机
    //--------------------------------------------------------------------------

    /**
     * @brief 判停阈值(线速度)
     * @unit m/s
     * @note  不能用 "== 0.0f"：编码器有量化与回差，浮点几乎永远不正好是 0
     */
    constexpr float STOP_LINEAR_EPS = 0.02f;

    /**
     * @brief 判停阈值(角速度)
     * @unit rad/s
     */
    constexpr float STOP_ANGULAR_EPS = 0.05f;

    /**
     * @brief 判停超时
     * @unit ms
     * @note  超过这个时间无论是否判定为静止都结束刹车流程。
     *        用于兜住"编码器失效"、"车被顶住"、"坡道上溜车"这类判不出来情况，
     *        保证状态机不会永远停在 kCoasting / kBraking
     */
    constexpr uint32_t STOP_TIMEOUT_MS = 500U;

    /**
     * @brief 进入刹车态后的最小驻留时间
     * @unit ms
     * @note  测速窗口有零阶保持，刚进刹车态时可能还没填满，会给出几百毫秒的 0；
     *        若此时就看 isChassisStopped()，会在车还在高速滑行时被误判成"已停"。
     *        取值应不小于 2 个测速窗口(MecanumParam::SPEED_WINDOW_S)
     */
    constexpr uint32_t STOP_MIN_DWELL_MS = 20U;
}

/**
 * @brief 底盘速度
 * @note  既用于世界系平移目标，也用于车体系/世界系的实际速度读数；
 *        omega 是绕 z 轴的角速度，平面运动下它在两个坐标系里数值相同
 */
struct ChassisVelocity {
    float vx;    /**< 前后方向线速度 (单位: m/s, 向前为正) */
    float vy;    /**< 左右方向线速度 (单位: m/s, 向左为正) */
    float omega; /**< 自旋角速度 (单位: rad/s, 逆时针为正，与 IMU yaw 同向) */

    /**
     * @brief 默认构造，全部置零(静止)
     */
    ChassisVelocity() : vx(0.0f), vy(0.0f), omega(0.0f) {}

    /**
     * @brief 依次构造
     * @param vx_ 前后速度 (m/s, 向前为正)
     * @param vy_ 左右速度 (m/s, 向左为正)
     * @param omega_ 自旋角速度 (rad/s, 逆时针为正)
     */
    ChassisVelocity(float vx_, float vy_, float omega_)
        : vx(vx_), vy(vy_), omega(omega_) {}
};

/**
 * @brief 角速度环的控制权归属
 *
 * @note  角速度这条执行通道是**独占**的：同一时刻只允许一方给它下指令。
 *        旧实现把"航迹方向修正量"与"自旋指令"相加，结果是指令自旋时
 *        航迹环会把旋转拉回去，两个目标永远互相打架。
 *
 * @note  控制权的夺取与归还：
 *        - setCourseTarget()  -> 夺取，变为 kCourse
 *        - setSpinTarget()    -> 夺取，变为 kSpin
 *        - releaseAngularControl() -> 归还，变为 kIdle
 *        夺取是"后来者胜出"，被顶掉的一方指令立即失效并被清零。
 */
enum class AngularControl : uint8_t {
    kIdle = 0,   /**< 无人持有：角速度指令恒为 0，车体朝向不受控(但平移仍然生效) */
    kCourse,     /**< 航迹方向持有：角速度环以 course_target_rad_ 为目标，维持航迹方向 */
    kSpin        /**< 自旋持有：角速度环以 spin_target_radps_ 为目标，只控自旋速度 */
};

/**
 * @brief   麦克纳姆轮底盘运动学解算与控制类，对外公开底盘当前运动状态。
 *          速度环(四轮) + 角速度环(单路，两用) 双闭环。
 * @note    依赖：IWheelDriver / IHeadingSensor 抽象接口(见 core/interfaces/)、PID类(闭环计算)。
 *          不直接依赖任何具体硬件类，便于用仿真实现做离线测试与回归测试
 * @note    坐标与符号约定(全局统一，与 IMU 保持一致)：
 *          x 轴向前、y 轴向左、z 轴向上，构成右手系；
 *          vx 向前为正，vy 向左为正，omega 逆时针(俯视)为正；
 *          世界系的原点方向 = init()/calibrationIMU() 时刻的车头朝向
 * @note    两个坐标系(务必区分，混用会导致"世界系航迹"变成"车体系航迹")：
 *          - 世界系：与车体朝向无关，上层下达的航迹方向与平移目标都在这里
 *          - 车体系：与车体固连，麦轮逆运动学**只能**在车体系下成立
 *          update() 每周期把世界系平移目标按当前 yaw 旋转到车体系再解算
 * @note    调用约定：
 *          1. 构造后先调用 init()，返回 true 才可调用其余接口
 *          2. update(dt) 与 stop(dt) 必须在固定周期的定时器中断或任务中调用，且二选一
 *          3. 角速度环的控制权由 AngularControl 表达，setCourseTarget() 与
 *             setSpinTarget() 互斥，同一时刻只有一方生效
 * @author  泠熙语云
 * @date    2026-10-14
 * @version 1.2
 */
class ChassisController {
    public:
    /**
     * @brief 构造底盘控制器
     * @param[in] wheel_driver 轮驱动硬件接口，不持有所有权，生命周期由调用方保证
     * @param[in] heading_sensor 航向传感器接口，不持有所有权
     * @note  构造函数不操作硬件，硬件初始化在 init() 中完成
     */
    ChassisController(IWheelDriver& wheel_driver, IHeadingSensor& heading_sensor);

    /**
     * @brief 初始化底盘控制器：初始化底层驱动、配置 PID 参数与限幅、复位内部状态
     * @return true 初始化成功，false 底层硬件初始化失败
     * @note  会把 world_coordinate_system_reference_course 清零(即把当前车头方向
     *        定义为世界系 +x)，并把角速度环置为 kIdle
     */
    bool init();

    /**
     * @brief 校准IMU，并把标定结束时刻的车头方向重新定义为世界系 +x
     * @return true 校准成功，false 校准失败
     * @note 调用前应确保底盘静止
     * @note 会顺带把航迹目标对齐到新的 +x，避免标定之后车自己转一下
     */
    bool calibrationIMU();

    /**
     * @brief 查询控制器是否可用
     * @return true 已成功初始化
     */
    bool isReady() const;

    /**
     * @brief 使能/失能闭环输出
     * @param[in] enabled true 使能闭环，false 停止输出
     * @note  失能时会同步失能轮驱动；重新使能时会 reset() 各路 PID，
     *        避免带旧积分量输出突变
     * @note  使能状态与角速度环的控制权是两件独立的事：
     *        失能不会改变 AngularControl，重新使能后原来的模式继续生效
     * @note  失能会落到**滑行态**，并清掉正在进行的刹车状态。
     *        也就是说 `setEnabled(false)` 会让已经施加的绕组短接急刹**松开**；
     *        需要"停住并保持刹住"请用 emergencyStop()
     */
    void setEnabled(bool enabled);

    /**
     * @brief 查询闭环使能状态
     * @return true 已使能
     */
    bool isEnabled() const;

    //==========================================================================
    // 运动目标下达：平移通道 与 角速度通道 分开
    //==========================================================================

    /**
     * @brief 设置世界坐标系下的目标航迹(方向 + 速度大小)，并**夺取**角速度环控制权
     * @param[in] course_rad 目标航迹方向，单位 rad，逆时针为正，
     *                       0 = init()/calibrationIMU() 时刻的车头朝向
     * @param[in] speed_mps  沿该方向的速度大小，单位 m/s；
     *                       负值按 0 处理(要反向请把 course_rad 加 π)，
     *                       超过 MAX_TRANSLATE_SPEED 按上限处理
     * @return true 已采纳；false 控制器未就绪(isReady() == false)
     *
     * @note  【互斥】与 setSpinTarget() 互斥。本函数成功后 AngularControl 变为
     *        kCourse，之前由 setSpinTarget() 设定的自旋指令立刻失效并被清零。
     *        若控制权确实发生了切换，角速度环 PID 会被 reset()，不会带着
     *        旧模式的积分量输出。
     *
     * @note  内部把 (course_rad, speed_mps) 分解成世界系 (vx, vy) 存储：
     *        vx = speed*cos(course)、vy = speed*sin(course)，
     *        再由 update() 按当前 yaw 旋转到车体系后送入逆运动学。
     *
     * @note  可以在控制周期里重复调用(例如持续下发摇杆指令)：
     *        模式没变时只更新目标，不会 reset() PID，因此积分量不会被打断。
     */
    bool setCourseTarget(float course_rad, float speed_mps);

    /**
     * @brief 设置自旋角速度目标，并**夺取**角速度环控制权
     * @param[in] omega_radps 目标自旋角速度，单位 **rad/s**，逆时针为正；
     *                        超过 MAX_SPIN_RATE 按上限处理
     * @return true 已采纳；false 控制器未就绪
     *
     * @note  【互斥】与 setCourseTarget() 互斥。本函数成功后 AngularControl 变为
     *        kSpin，航迹方向的修正量立刻失效，车体不再维持航迹方向
     *        (course_target_rad_ 保留原值但不再参与计算，下一次
     *        setCourseTarget() 会覆盖它)。
     *
     * @note  平移目标**保持不变**。因此可以做到"一边自旋、一边沿固定的世界方向平移"：
     *        update() 仍会用当前 yaw 把世界系平移目标旋转到车体系。
     */
    bool setSpinTarget(float omega_radps);

    /**
     * @brief 归还角速度环控制权(角速度指令归零)，不改变平移目标
     * @note  会把航迹目标对齐到当前朝向，之后重新进入 kCourse 时不会突然转向
     */
    void releaseAngularControl();

    /**
     * @brief 查询当前角速度环由谁持有
     * @return AngularControl 控制权归属
     */
    AngularControl getAngularControl() const;

    /**
     * @brief 只设置世界系平移目标，**不改变**角速度环控制权
     * @param[in] vx_world 世界系 x 方向速度 (m/s)
     * @param[in] vy_world 世界系 y 方向速度 (m/s)
     * @note  给遥测/测试/遥控用：它不会夺取也不会归还控制权，
     *        因此可以与 setCourseTarget() 或 setSpinTarget() 叠加使用
     */
    void setWorldTranslateTarget(float vx_world, float vy_world);

    //==========================================================================
    // 状态读取
    //==========================================================================

    /**
     * @brief 读取世界系平移目标
     * @return ChassisVelocity 其中 omega 恒为 0(角速度目标不在这里，见 getSpinTarget())
     */
    ChassisVelocity getTranslateTargetWorld() const;

    /**
     * @brief 读取航迹方向目标
     * @return 目标航迹方向，单位 rad；仅在 AngularControl == kCourse 时有意义
     */
    float getCourseTarget() const;

    /**
     * @brief 读取自旋角速度目标
     * @return 目标角速度，单位 rad/s；仅在 AngularControl == kSpin 时有意义
     */
    float getSpinTarget() const;

    /**
     * @brief 底盘当前运动状态(车体系)
     * @return ChassisVelocity 车体系速度
     * @note 数据来自快照，读取是原子的
     */
    ChassisVelocity getCurrentVelocity() const;

    /**
     * @brief 底盘当前运动状态(世界系)
     * @return ChassisVelocity 世界系速度，与 setCourseTarget() 同一坐标系
     * @note 单次调用内部是原子的，不会读到写了一半的数据；但它与
     *       getCurrentVelocity() 是两次独立取快照，中间若正好跑了一次 update()，
     *       二者会来自不同的采样时刻。需要严格同源时，只用其中一个坐标系即可
     */
    ChassisVelocity getCurrentVelocityWorld() const;

    /**
     * @brief 查询速度反馈数据是否有效
     * @return true 已至少完成一次有效解算
     * @note 用于区分"底盘确实静止"与"还没跑过 update() 所以速度是 0"
     */
    bool isDataValid() const;

    //==========================================================================
    // 参数配置
    //==========================================================================

    /**
     * @brief 设置四路速度环PID参数
     * @param kp 比例
     * @param ki 积分
     * @param kd 微分
     * @note 四路使用同一组参数；仅调试整定时使用，运行中修改应配合 reset()
     */
    void setWheelPidParameters(float kp, float ki, float kd);

    /**
     * @brief 设置航迹模式的角速度环 PID 参数
     * @param kp 比例(量纲 1/s)
     * @param ki 积分
     * @param kd 微分
     * @note 若当前正处于 kCourse，会立刻生效并 reset() 积分量
     */
    void setCoursePidParameters(float kp, float ki, float kd);

    /**
     * @brief 设置自旋模式的角速度环 PID 参数
     * @param kp 比例
     * @param ki 积分
     * @param kd 微分
     * @note 若当前正处于 kSpin，会立刻生效并 reset() 积分量
     */
    void setSpinPidParameters(float kp, float ki, float kd);

    /**
     * @brief 设置单轮转速上限
     * @param[in] rps 单轮转速上限，单位 r/s，必须 > 0(非正值会被忽略)
     */
    void setMaxWheelSpeed(float rps);

    /**
     * @brief 读取单轮转速上限
     * @return 单轮转速上限，单位 r/s
     */
    float getMaxWheelSpeed() const;

    /**
     * @brief 设置平移速度上限
     * @param[in] mps 平移速度上限，单位 m/s，必须 > 0(非正值会被忽略)
     */
    void setMaxTranslateSpeed(float mps);

    /**
     * @brief 读取平移速度上限
     * @return 平移速度上限，单位 m/s
     */
    float getMaxTranslateSpeed() const;

    /**
     * @brief 设置自旋角速度上限
     * @param[in] radps 自旋角速度上限，单位 rad/s，必须 > 0(非正值会被忽略)
     */
    void setMaxSpinRate(float radps);

    /**
     * @brief 读取自旋角速度上限
     * @return 自旋角速度上限，单位 rad/s
     */
    float getMaxSpinRate() const;

    //==========================================================================
    // 停机
    //==========================================================================

    /**
     * @brief 平滑停机：清零平移目标并归还角速度环，保持驱动使能
     * @note 与 emergencyStop() 的区别：本函数不改变使能状态
     */
    void normalstop();

    /**
     * @brief 紧急停机：立即失能驱动并**同步**施加绕组短接急刹，同时清空所有 PID 状态
     * @note  用于失控、通信中断等异常场景；恢复需要重新 setEnabled(true)
     * @note  急刹是同步生效的，不依赖 stop() 被周期性调用；之后由 stop() 负责
     *        判停/超时收尾(收尾时若仍是失能状态，会**保持**刹住不松开)
     */
    void emergencyStop();

    /**
     * @brief 刹车状态机(自由滑行 / 绕组短接急刹)
     * @param dt 本次调用距上次调用的时间间隔，单位 s，必须 > 0
     * @note  与 update(dt) 是**互斥入口**：刹车流程进行中不要再调 update()，
     *        否则速度环会和刹车抢同一路输出
     * @note  进入 kCoasting / kBraking 后，本函数会**自己**读取轮速反馈，
     *        不依赖 update() 是否跑过
     * @note  本函数只依赖轮驱动，**不要求 IMU 已校准**：
     *        保证在任何情况下(包括 IMU 掉线、未标定)都能把车停下来
     * @note  判停用阈值(ChassisParam::STOP_LINEAR_EPS / STOP_ANGULAR_EPS)
     *        而不是与 0.0f 精确比较；另有 ChassisParam::STOP_TIMEOUT_MS
     *        超时兜底，保证状态机一定能退出，不会因为编码器异常或车被顶住而卡死
     * @note  流程结束时若驱动处于**失能**状态，本函数不会去恢复 PWM 通路，
     *        而是把输出留在滑行/刹车电平上(急刹之后即保持刹住)，
     *        恢复由下一次 setEnabled(true) 负责
     */
    void stop(float dt);

    /**
     * @brief 底盘闭环控制主更新函数
     * @details 执行顺序：
     *          1. 读 IMU，算出相对世界系的当前 yaw
     *          2. 角速度通道：按 AngularControl **择一**取指令
     *             - kCourse：角度误差 -> 角速度环 -> 角速度修正量
     *             - kSpin  ：角速度误差 -> 角速度环 -> 角速度指令
     *             - kIdle  ：角速度指令恒 0
     *             (注意这里是"赋值"，不是把两路相加)
     *          3. 平移通道：世界系 (vx, vy) 按当前 yaw 旋转到车体系
     *          4. 逆运动学：车体系 (vx, vy, omega) 分解为四轮目标转速(r/s)
     *          5. 全轮等比限幅：超过 max_wheel_speed_ 时四轮按同一比例缩小
     *          6. 读取反馈：wheel_driver_.getWheelSpeed(dt)
     *          7. 四路速度环 PID：目标轮速 vs 实际轮速 -> PWM
     *          8. 正运动学：刷新车体系/世界系两份速度快照与 data_valid_
     * @param dt 本次调用距上次调用的时间间隔，单位 s，必须 > 0
     * @note  必须在定时器中断或固定周期的任务中调用，且周期必须与 dt 一致
     * @note  未使能或未初始化时不产生 PWM 输出
     * @note  全轮等比限幅会**同时**缩放平移与角速度两个通道：平移速度接近上限时，
     *        航迹模式的角速度修正量也会被一起压小，表现为"跑得越快越转不动"。
     *        这是等比限幅的固有代价(为了保证四轮比例、方向不偏)，调参时
     *        要按实际最大平移速度给角速度留足余量
     */
    void update(float dt);

private:
    /**
     * @brief 逆运动学解算，将车体系速度分解为四轮目标转速
     * @param[in] vel 车体系速度(m/s, rad/s)
     * @return 四轮目标转速 (r/s)，轮序 LF, RF, LR, RR(见 WheelIndex)
     * @note  本函数不做限幅，限幅由 limitWheelSpeed() 负责
     * @note  麦轮逆运动学只在**车体系**下成立，传入前必须先完成世界系->车体系的旋转
     */
    WheelSpeed inverseKinematics(const ChassisVelocity& vel) const;

    /**
     * @brief 四轮转速等比限幅
     * @param[in] cmd 逆解算得到的四轮目标转速
     * @return 限幅后的四轮目标转速；若有任一轮超限，四轮按同一比例缩放，
     *         保持速度方向与四轮比例关系不变
     */
    WheelSpeed limitWheelSpeed(const WheelSpeed& cmd) const;

    /**
     * @brief 正运动学解算，由四轮实际转速反推车体系实际速度
     * @param[in] wheel_speeds 四轮实际转速 (r/s)
     * @return 车体系实际速度
     */
    ChassisVelocity forwardKinematics(const WheelSpeed& wheel_speeds) const;

    /**
     *@brief PID-PWM 转换函数，带符号
     *@param pidCorrection PID修正力度（-1.0 ~ 1.0）
     *@return PWM 占空比计数值，带符号
     */
    int16_t PID_CorrectionToPwm(float pidCorrection);

    /**
     * @brief 角速度环控制权的唯一切换入口
     * @param[in] next 目标控制权归属
     * @note  只有**模式真的改变**时才 reset() PID 并重装增益；
     *        同一模式重复调用是空操作，这样上层可以在控制周期里持续下发指令
     */
    void switchAngularControl(AngularControl next);

    /**
     * @brief 按控制权归属装载角速度环的增益与输出限幅
     * @param[in] mode 目标模式
     */
    void applyAngularGains(AngularControl mode);

    /**
     * @brief 读取相对世界系的当前 yaw
     * @return 航向角 rad，逆时针为正，0 = 世界系 +x
     */
    float currentYawWorld();

    /**
     * @brief 请求退出刹车状态
     * @note  只在当前确实处于刹车流程时才置 kTermination，
     *        避免上层每个周期下发指令时把刹车状态反复改写
     */
    void requestResumeFromBrake();

    /**
     * @brief 判断底盘是否已经停住(用阈值，不用精确等于 0)
     * @return true 线速度与角速度的绝对值都在判停阈值以内
     */
    bool isChassisStopped() const;

    IWheelDriver& wheel_driver_;        /**< 轮驱动硬件接口，不持有所有权 */
    IHeadingSensor& heading_sensor_;    /**< 航向传感器接口，不持有所有权 */

    PID wheel_pid_[kWheelCount];        /**< 四路速度环PID，输出为修正量 -1.0~1.0 */
    PID angular_pid_;                   /**< 角速度环PID，两种模式共用，同一时刻只喂一方 */

    ChassisVelocity velocity_target_world_;   /**< 世界系平移目标，由 set*Target() 更新 */
    ChassisVelocity velocity_current_;        /**< 车体系实际速度快照，由 update() 刷新 */
    ChassisVelocity velocity_current_world_;  /**< 世界系实际速度快照，与上一行同时写入 */

    float course_target_rad_;           /**< 航迹方向目标 (rad)，由 setCourseTarget() 更新 */
    float spin_target_radps_;           /**< 自旋角速度目标 (rad/s)，由 setSpinTarget() 更新 */

    // 注意命名：本类里 _rps 一律是"转/秒"(轮速)，角速度一律用 _radps。
    // 两者都是"每秒"，混用会让 2π 这个因子凭空消失。
    float max_wheel_speed_;             /**< 单轮转速上限 (rps = 转/秒) */
    float max_translate_speed_;         /**< 平移速度上限 (m/s) */
    float max_spin_rate_;               /**< 自旋角速度上限 (rad/s) */
    float max_omega_correction_;        /**< 航迹模式角速度环输出上限 (rad/s) */

    float course_kp_ = 0.0f;            /**< 航迹模式增益(量纲 1/s) */
    float course_ki_ = 0.0f;
    float course_kd_ = 0.0f;
    float spin_kp_ = 0.0f;              /**< 自旋模式增益 */
    float spin_ki_ = 0.0f;
    float spin_kd_ = 0.0f;

    float world_coordinate_system_reference_course = 0.0f; /**< 世界坐标系基准航向(rad) */

    AngularControl angular_control_ = AngularControl::kIdle; /**< 角速度环当前由谁持有 */
    bool enabled_ = false;              /**< 闭环是否使能 */
    bool data_valid_ = false;           /**< 速度快照是否已产生过有效数据 */
    BrakeMode brake_mode_ = BrakeMode::k_; /**< 刹车状态机当前状态 */
    uint32_t brake_tick_ = 0;           /**< 进入刹车状态时的 HAL_GetTick() 时刻(ms) */
};


#endif //A_TEST_CHASSISCONTROLLER_H
