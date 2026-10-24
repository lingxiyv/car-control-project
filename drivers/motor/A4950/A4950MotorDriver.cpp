//
// Created by lingxi on 2026/10/14.
//

#include "A4950MotorDriver.h"

// 板级唯一的对外契约：引脚宏与外设句柄都从这里来。
// 不直接 include Core/Inc/tim.h / main.h / gpio.h。
#include "BoardPins.h"

namespace {

/**
 * @brief 4 路 PWM 通道，顺序固定为 LF, RF, LR, RR，与 WheelIndex 一致
 * @note  PA8/PA9/PA10/PA11 -> TIM1_CH1/CH2/CH3/CH4，
 *        分别对应 LF_IN2 / RF_IN2 / LR_IN2 / RR_IN2(见 BoardPins.h)
 */
constexpr uint32_t kPwmChannel[kWheelCount] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/**
 * @brief 逐个通道启动 PWM
 * @warning 不能用 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_ALL)
 *          HAL 内部 TIM_CCxChannelCmd() 用 CC1E << (Channel & 0x1F) 定位要写的位，
 *          TIM_CHANNEL_ALL(0x3C) 会被算成 CCER 的 bit28(保留位)，
 *          CC1E~CC4E 一个都不会被置位 —— 计数器在跑、MOE 也开了，
 *          但四个通道输出全部禁用，PA8~PA11 上没有任何波形，且函数照样返回 HAL_OK。
 *          (只有 HAL_TIM_Encoder_Start 内部把 TIM_CHANNEL_ALL 展开了，PWM 系列没有)
 */
void pwmStartAll() {
    for (uint32_t ch : kPwmChannel) {
        HAL_TIM_PWM_Start(&htim1, ch);
    }
}

/**
 * @brief 逐个通道停止 PWM
 * @note  __HAL_TIM_DISABLE / __HAL_TIM_MOE_DISABLE 都是"所有 CCxE 都关掉才生效"的
 *        条件宏，所以必须逐个通道停，最后一个通道停下时才会清 CEN 与 MOE
 */
void pwmStopAll() {
    for (uint32_t ch : kPwmChannel) {
        HAL_TIM_PWM_Stop(&htim1, ch);
    }
}

/**
 * @brief  单轮输出：IN1(普通 GPIO)静态定方向，IN2(TIM1_CHx) 承载 PWM 占空比
 * @param  dir_port/dir_pin 该轮的 IN1
 * @param  ch               该轮 IN2 所对应的定时器通道
 * @param  cmd              PWM 指令，正数正转、负数反转，内部钳位到 ±PWM_MAX
 *
 * @note   A4950 真值表：IN1=H,IN2=H -> 刹车(绕组短接)；IN1=L,IN2=L -> 滑行(输出高阻)；
 *         IN1=H,IN2=L -> 正转；IN1=L,IN2=H -> 反转。
 *
 *         因为 IN1 只有静态电平、只有 IN2 能出 PWM，两个方向天生是两种调制方式：
 *          - 反转(IN1=L)：IN2 高电平段=反转、低电平段=滑行 -> 平均电压 ∝ d      (正确)
 *          - 正转(IN1=H)：IN2 高电平段=刹车、低电平段=正转 -> 平均电压 ∝ (1-d)  (极性反了)
 *         所以正转必须写补码 CCR = PWM_MAX - cmd，"指令越大转速越高"才成立。
 *         若直接写 CCR = cmd，正转的闭环增益是负的：PID 给正输出、轮子反而变慢，
 *         误差继续变大，最后锁死在满刹车。
 *
 *         零指令单独处理：落进正转分支会得到 CCR = PWM_MAX -> IN2 恒高
 *         -> IN1=H,IN2=H -> 满刹车；落进反转分支则是 CCR=0 -> 滑行。显式取滑行更安全。
 */
void applyWheelOutput(GPIO_TypeDef* dir_port, uint16_t dir_pin,
                      uint32_t ch, int16_t cmd) {
    // 接口契约要求实现侧钳位；同时正转分支的 PWM_MAX - duty 也依赖 duty 不越界，
    // 否则会算成负数再被当成无符号写进 CCR，变成满占空比
    int16_t duty = cmd;
    if (duty > MotorParam::PWM_MAX) {
        duty = MotorParam::PWM_MAX;
    } else if (duty < -MotorParam::PWM_MAX) {
        duty = -MotorParam::PWM_MAX;
    }

    if (duty > 0) {
        // 正转：IN1 拉高；IN2 写补码，使"正转"的时间占比 = duty / PWM_MAX
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, ch, MotorParam::PWM_MAX - duty);
    } else if (duty < 0) {
        // 反转：IN1 拉低；IN2 直接给占空比
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, ch, static_cast<uint32_t>(-duty));
    } else {
        // 零指令：显式滑行(IN1=L, IN2=L)
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
    }
}

} // namespace

A4950MotorDriver::A4950MotorDriver() {
    initialized_ = false;
    enabled_ = false;
    pwm_output_ready_ = false;
    for (uint8_t i = 0; i < kWheelCount; i++) {
        prev_count_[i] = 0;
    }
}

bool A4950MotorDriver::init() {
    /*电机驱动板为A4950，编码器为CDR-HM37-520自带，其初始化没有特别操作
     *唯一要做的就是使能（见setEnabled()）
     *连接至驱动板与电机的普通引脚初始化与定时器初始化由HAL库的底层函数完成
     *此处信任底层初始化
     */
    initialized_ = true;
    //PWM 通路要等 setEnabled(true) 里 RestorePWM_GPIO() 之后才可用
    pwm_output_ready_ = false;
    //测速状态初始化(此时编码器定时器可能还没启动，读 CNT 得到 0，无副作用)
    resetSpeedMeasurement();
    return true;
}

void A4950MotorDriver::deinit() {
    //把 4 路输出切到滑行态(IN1=L, IN2=L)并关闭 PWM
    enterStaticDrive(false);
    //停止所有编码器定时器(编码器接口本身支持 TIM_CHANNEL_ALL)
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim8, TIM_CHANNEL_ALL);
    //警告：以下 DeInit 会彻底销毁定时器句柄
    //如果后续还需要重新 init()，必须先重新调用 CubeMX 生成的 MX_TIMx_Init()
    HAL_TIM_PWM_DeInit(&htim1);
    HAL_TIM_Encoder_DeInit(&htim2);
    HAL_TIM_Encoder_DeInit(&htim3);
    HAL_TIM_Encoder_DeInit(&htim4);
    HAL_TIM_Encoder_DeInit(&htim8);
    initialized_ = false;
    enabled_ = false;
}

bool A4950MotorDriver::isInitialized() const {
    return initialized_;
}

void A4950MotorDriver::setEnabled(bool enabled) {
    //未初始化(含已 deinit)时不允许再动定时器：那时 TIM1 时钟已被关闭
    if (!initialized_) {
        return;
    }
    if (enabled) {
        //直接走恢复流程：内含"先摆到滑行态 + 切回 AF1_TIM1 + 逐通道启动"，
        //这样不管上一次是滑行、急刹还是失能，都能回到确定的可驱动状态
        RestorePWM_GPIO();
        //启动所有编码器定时器
        HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
        //重开测速窗口，避免把失能期间的转动算进第一拍
        resetSpeedMeasurement();
        enabled_ = true;
    }else {
        //失能必须落到"滑行态"，不能只停 PWM：
        //若 IN1 还残留着上一条正转指令的高电平，而 IN2 因为定时器停止/通道关闭变成低电平，
        //A4950 会得到 IN1=H, IN2=L —— 即满速正转(失能反而全速前进)
        enterStaticDrive(false);
        //注意这里**不停**编码器定时器：编码器是只读输入，停掉之后
        //getWheelSpeed() 会恒返回 0，停机状态机就再也判不出"车还在滑行"。
        //电机输出已经在 enterStaticDrive() 里断掉了，编码器继续跑没有副作用。
        //只有 deinit() 才会停它。
        enabled_ = false;
    }
}

bool A4950MotorDriver::isEnabled() const {
    return enabled_;
}

void A4950MotorDriver::setPwm(const WheelPwm& pwm) {
    //pwm_output_ready_ 很关键：滑行/急刹之后 PA8~PA11 已经切成静态 GPIO，
    //此时 CCR 写不进去、IN2 被钉死，若还继续只改 IN1，就会出现
    //IN1=H,IN2=L(满速正转) 或 IN1=L,IN2=H(满速反转)
    if (!initialized_ || !enabled_ || !pwm_output_ready_) {
        return;
    }
    //通道映射必须与 BoardPins.h 的 IN2 引脚一致：
    //LF->PA8(CH1)  RF->PA9(CH2)  LR->PA10(CH3)  RR->PA11(CH4)
    applyWheelOutput(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, TIM_CHANNEL_1, pwm.lf);
    applyWheelOutput(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, TIM_CHANNEL_2, pwm.rf);
    applyWheelOutput(MOTOR_LR_IN1_PORT, MOTOR_LR_IN1_PIN, TIM_CHANNEL_3, pwm.lr);
    applyWheelOutput(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN, TIM_CHANNEL_4, pwm.rr);
}

WheelSpeed A4950MotorDriver::getWheelSpeed(float dt) {
    //未初始化(含已 deinit)时读编码器寄存器没有意义：deinit 之后定时器时钟已关闭。
    //dt 非法时保持上一次结果，避免除零
    if (!initialized_ || dt <= 0.0f) {
        return speed_hold_;
    }

    //读取硬件寄存器(每路只读一次，避免两次读取之间计数前进导致丢拍)
    const uint16_t cnt_lf = ENCODER_LF->CNT;
    const uint16_t cnt_rf = ENCODER_RF->CNT;
    const uint16_t cnt_lr = ENCODER_LR->CNT;
    const uint16_t cnt_rr = ENCODER_RR->CNT;

    //计算差值，并乘上方向符号修正 A/B 相接反的轮子(占位值，见 MecanumParam)
    const int32_t delta_lf = calc_encoder_delta(cnt_lf, prev_count_[0]) * MecanumParam::WHEEL_DIRECTION[kWheelLf];
    const int32_t delta_rf = calc_encoder_delta(cnt_rf, prev_count_[1]) * MecanumParam::WHEEL_DIRECTION[kWheelRf];
    const int32_t delta_lr = calc_encoder_delta(cnt_lr, prev_count_[2]) * MecanumParam::WHEEL_DIRECTION[kWheelLr];
    const int32_t delta_rr = calc_encoder_delta(cnt_rr, prev_count_[3]) * MecanumParam::WHEEL_DIRECTION[kWheelRr];

    //更新历史计数
    prev_count_[0] = cnt_lf;
    prev_count_[1] = cnt_rf;
    prev_count_[2] = cnt_lr;
    prev_count_[3] = cnt_rr;

    //累计进当前测速窗口
    speed_delta_acc_[kWheelLf] += delta_lf;
    speed_delta_acc_[kWheelRf] += delta_rf;
    speed_delta_acc_[kWheelLr] += delta_lr;
    speed_delta_acc_[kWheelRr] += delta_rr;
    speed_window_acc_s_ += dt;

    //窗口还没满：保持上一次结果(零阶保持)，不拿噪声去喂速度环
    if (speed_window_acc_s_ < MecanumParam::SPEED_WINDOW_S) {
        return speed_hold_;
    }

    //窗口满了：用窗口内的累计计数与累计时长算转速
    //公式：速度(rps) = 累计计数 / (编码器线数 * 4 * 减速比) / 窗口时长
    const float inv = 1.0f / (speed_window_acc_s_ * MecanumParam::COUNTS_PER_WHEEL_REV);
    speed_hold_ = WheelSpeed(speed_delta_acc_[kWheelLf] * inv,
                             speed_delta_acc_[kWheelRf] * inv,
                             speed_delta_acc_[kWheelLr] * inv,
                             speed_delta_acc_[kWheelRr] * inv);

    //重新开一个窗口
    for (uint8_t i = 0; i < kWheelCount; i++) {
        speed_delta_acc_[i] = 0;
    }
    speed_window_acc_s_ = 0.0f;

    return speed_hold_;
}

void A4950MotorDriver::resetSpeedMeasurement() {
    //重读一次计数作为新基准，避免把失能期间的转动算进第一拍
    prev_count_[0] = ENCODER_LF->CNT;
    prev_count_[1] = ENCODER_RF->CNT;
    prev_count_[2] = ENCODER_LR->CNT;
    prev_count_[3] = ENCODER_RR->CNT;
    for (uint8_t i = 0; i < kWheelCount; i++) {
        speed_delta_acc_[i] = 0;
    }
    speed_window_acc_s_ = 0.0f;
    speed_hold_ = WheelSpeed();
}

void A4950MotorDriver::CoastStop_WithStartupDelay_PWMDisabled() {
    enterStaticDrive(false);
}

void A4950MotorDriver::BrakeStop() {
    enterStaticDrive(true);
}

void A4950MotorDriver::enterStaticDrive(bool brake) {
    const GPIO_PinState level = brake ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // 0) 标记 PWM 通路不可用：此后 setPwm() 会自动失效，
    //    避免"IN2 被钉死、只改 IN1"凑出满速正转/反转
    pwm_output_ready_ = false;

    // 1) 先停 PWM：清 CCxE、关 MOE、停计数器。停完之后通道输出为无效电平(低)
    pwmStopAll();
    for (uint32_t ch : kPwmChannel) {
        __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
    }

    // 2) 先把方向脚 IN1 摆到位。IN1 是普通 GPIO、立刻生效，
    //    而此时 IN2 还是定时器输出(已停、CCR=0 -> 低)，
    //    所以"正转 -> 滑行"这种最常见的路径不会出现中间态
    HAL_GPIO_WritePin(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, level);
    HAL_GPIO_WritePin(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, level);
    HAL_GPIO_WritePin(MOTOR_LR_IN1_PORT, MOTOR_LR_IN1_PIN, level);
    HAL_GPIO_WritePin(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN, level);

    // 3) 再把 IN2 的 ODR 摆到目标电平(此时引脚仍由 AF 驱动，ODR 只是锁存)
    HAL_GPIO_WritePin(MOTOR_LF_IN2_PORT, MOTOR_LF_IN2_PIN, level);
    HAL_GPIO_WritePin(MOTOR_RF_IN2_PORT, MOTOR_RF_IN2_PIN, level);
    HAL_GPIO_WritePin(MOTOR_LR_IN2_PORT, MOTOR_LR_IN2_PIN, level);
    HAL_GPIO_WritePin(MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN, level);

    // 4) PA8~PA11 从 AF1_TIM1 切成普通推挽输出，引脚立即按上面锁存的 ODR 输出。
    //    注意必须是 GPIO_MODE_OUTPUT_PP。若误写成 GPIO_MODE_AF_PP 且不填 Alternate，
    //    AFR 会被写成 AF0(0 号复用功能)，引脚仍由外设驱动，
    //    后面 HAL_GPIO_WritePin() 只改 ODR、对引脚电平完全无效
    GPIO_InitTypeDef gpio_conf = {0};
    gpio_conf.Pin   = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio_conf.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_conf.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_conf.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_conf);
}

void A4950MotorDriver::RestorePWM_GPIO() {
    // 1) 先摆到安全的滑行态：IN1 全低、IN2 全低、CCR 清零。
    //    否则若刚做过急刹(IN1 残留为高)，恢复 PWM 且 CCR=0 时得到 IN1=H,IN2=L，
    //    会瞬间变成满速正转
    HAL_GPIO_WritePin(MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LR_IN1_PORT, MOTOR_LR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(MOTOR_LF_IN2_PORT, MOTOR_LF_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RF_IN2_PORT, MOTOR_RF_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LR_IN2_PORT, MOTOR_LR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN, GPIO_PIN_RESET);

    for (uint32_t ch : kPwmChannel) {
        __HAL_TIM_SET_COMPARE(&htim1, ch, 0);
    }

    // 2) PA8~PA11 切回 AF1_TIM1
    GPIO_InitTypeDef gpio_conf = {0};
    gpio_conf.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio_conf.Mode      = GPIO_MODE_AF_PP;
    gpio_conf.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_conf.Pull      = GPIO_NOPULL;
    gpio_conf.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio_conf);

    // 3) 逐通道启动(不能用 TIM_CHANNEL_ALL，原因见 pwmStartAll 的注释)
    pwmStartAll();

    // 4) 通路恢复，setPwm() 重新生效
    pwm_output_ready_ = true;
}

int32_t A4950MotorDriver::calc_encoder_delta(uint16_t current, uint16_t prev) {
    int32_t diff = static_cast<int32_t>(current) - static_cast<int32_t>(prev);
    // 16位定时器的半量程是 32768
    if (diff > 32767) {
        diff -= 65536;
    } else if (diff < -32768) {
        diff += 65536;
    }
    return diff;
}
