//
// Created by lingxi on 2026/10/14.
//


#ifndef DRIVERS_MOTOR_A4950_A4950MOTORDRIVER_H
#define DRIVERS_MOTOR_A4950_A4950MOTORDRIVER_H

#include "IWheelDriver.hpp"

/**
 * @brief 麦轮驱动层硬件参数
 * @note  这些参数属于"驱动/编码器"层，用于把编码器计数换算成轮轴转速；
 *        机构参数(轮半径、轮距、轴距)在 ChassisParam 中。
 *        两者不要混放，否则驱动层要用机构参数就得反向包含控制器头文件
 */
namespace MecanumParam {
    /**
     *@brief 编码器线数
     */
    constexpr float ENCODER_RESOLUTION = 11.0f;

    /**
     * @brief 电机减速比
     */
    constexpr float GEAR_RATIO = 30.0f;

    /**
     * @brief 编码器单圈脉冲数(电机轴)，按实际计数方式(是否倍频)填写
     * @note 电机轴单圈计数 = 线数 * 4(TI12 四倍频)；轮轴单圈计数 = 再乘减速比
     */
    constexpr float COUNTS_PER_WHEEL_REV = (ENCODER_RESOLUTION * 4.0f) * GEAR_RATIO;

    /**
     * @brief 每轮编码器方向符号，用于修正 A/B 相接反
     * @note  【占位值】四路编码器分别接在 TIM2/3/4/8 上，A/B 相接线不一定一致，
     *        某个轮子读出来可能是反的。四轮方向不一致时正运动学算出的
     *        vx/vy/omega 会互相抵消，表现为"车在走但读数是乱的"。
     * @note  标定方法(批次2)：悬空该轮、给一个正 PWM，看 getWheelSpeed() 该轮读数；
     *        读数为负就把对应项改成 -1。顺序固定为 LF, RF, LR, RR。
     */
    constexpr int8_t WHEEL_DIRECTION[kWheelCount] = {+1, +1, +1, +1};

    /**
     * @brief 测速窗口长度(累计多长时间算一次转速)
     * @unit s
     * @note  编码器分辨率有限：轮轴每转只有 1320 个计数。1 ms 控制周期下跑 5 rps 时，
     *        每拍只有约 6.6 个计数，1 个计数的抖动就相当于 0.76 rps(约 15%)，
     *        直接喂给速度环的微分项会被噪声淹没。
     *        改成按窗口累计(默认 5 ms)后再算，5 rps 时约有 33 个计数，抖动降到约 3%。
     * @note  代价是速度反馈带宽降到 1/窗口 ≈ 200 Hz，窗口之间保持上次结果(零阶保持)。
     *        速度环带宽通常只有几十 Hz，够用；但 kd 要按新带宽重调。
     */
    constexpr float SPEED_WINDOW_S = 0.005f;
}

/**
 *@brief    麦克纳姆轮底盘底层硬件驱动类
 *          实现 IWheelDriver 接口：4 路电机 PWM 输出、编码器转速读取、电机与编码器使能控制
 *          只负责操作硬件寄存器，不做任何运动学解算
 *@note     init()需要在系统HAL初始化完成后调用一次
 *@warning  未成功初始化前调用其余接口无效
 *@note     IMU 不在此类中：航向传感器是底盘级设备，接口见 IHeadingSensor(IHeadingSensor.hpp)
 *@author   泠熙语云
 *@date     2026-10-14
 *@version  1.1
 */
class A4950MotorDriver : public IWheelDriver {
    public:
    /**
     * @brief 初始化内部变量
     * @note  不在此处操作硬件，硬件初始化在 init() 中完成
     */
    A4950MotorDriver();

    /**
     *@brief   初始化电机、编码器外设
     * @return true硬件初始化成功，false 硬件初始化失败
     */
    bool init() override;

    /**
     * @brief 反初始化，关闭 PWM 输出并释放外设资源
     * @warning 会彻底销毁定时器句柄
     *          如果后续还需要重新 init()，必须先重新调用 CubeMX 生成的 MX_TIMx_Init()
     */
    void deinit() override;

    /**
     * @brief 查询是否已初始化成功
     * @return true 已初始化
     */
    bool isInitialized() const override;

    /**
     * @brief 使能/失能电机输出
     * @param[in] enabled true 使能，false 失能
     * @note  失能后 setPwm() 不再产生实际输出
     * @note  失能**不**停止编码器：编码器是只读输入，停掉它会让 getWheelSpeed()
     *        恒返回 0，停机状态机就判不出"车还在滑行"
     */
    void setEnabled(bool enabled) override;

    /**
     * @brief 查询使能状态
     * @return true 已使能
     */
    bool isEnabled() const override;

    /**
     * @brief 设置四个轮子PWM输出
     * @param[in] pwm 四轮PWM目标值，值域[-MotorParam::PWM_MAX, MotorParam::PWM_MAX]，正负控制转向
     * @note  PWM 与转速是两种不同的物理量，故使用独立的 WheelPwm 类型，不要再用转速结构体承载
     */
    void setPwm(const WheelPwm& pwm) override;

    /**
     * @brief 读取四个轮子实时转速
     * @param[in] dt 距上一次调用本函数的时间间隔，单位 s，必须 > 0
     * @return WheelSpeed 结构体，四个轮子转速，单位r/s
     */
    WheelSpeed getWheelSpeed(float dt) override;

    /**
     * @brief 滑行停机
     * @note 会导致A4950休眠，再次启动出现延时
     * @warning 执行时会关闭 PWM 定时器，应在再启动时调用恢复
     */
    void CoastStop_WithStartupDelay_PWMDisabled() override;
    /**
     * @brief 主动停机
     * @warning 执行时会关闭 PWM 定时器，应在再启动时调用恢复
     */
    void BrakeStop() override;

    void RestorePWM_GPIO() override;

private:
    /**
     * @brief 把 4 路输出切到静态电平(关闭 PWM 输出)，供滑行与急刹共用
     * @param[in] brake true  = 四路 IN1/IN2 全高(A4950 绕组短接急刹)
     *                  false = 四路 IN1/IN2 全低(A4950 滑行/待机)
     * @note  会把 PA8~PA11 从 AF1_TIM1 切换成普通推挽输出，
     *        因此调用后必须再调用 RestorePWM_GPIO() 才能恢复 PWM 调速
     * @warning 不要把它写成 AF 模式：AF 模式下 HAL_GPIO_WritePin() 对引脚电平无效
     */
    void enterStaticDrive(bool brake);

    /**
     * @brief 重启测速窗口：重读计数基准、清空累计量
     * @note  使能时调用，避免把失能期间的转动或上一次的残留结果算进第一拍
     */
    void resetSpeedMeasurement();

    bool initialized_;                 /**< 外设是否初始化成功 */
    bool enabled_;                     /**< 电机与编码器是否使能 */
    /**
     * @brief PWM 通路是否处于可用状态(PA8~PA11 在 AF1_TIM1 且通道已启动)
     * @note  滑行/急刹会把 PA8~PA11 切成静态 GPIO，此时写入 CCR 没有任何效果，
     *        IN2 电平被"停"在一个固定值。若 setPwm() 这时仍然只改 IN1，
     *        就会得到 IN1=H,IN2=L 之类的组合 —— 也就是满速正转/反转。
     *        所以 setPwm() 必须靠这个标志把自己挡住。
     */
    bool pwm_output_ready_;
    uint16_t prev_count_[kWheelCount];  /**< 上一次采样的编码器计数，用于按 dt 差分求转速 */
    int32_t speed_delta_acc_[kWheelCount] = {0, 0, 0, 0}; /**< 当前测速窗口内累计的计数值 */
    float   speed_window_acc_s_ = 0.0f;                   /**< 当前测速窗口已累计的时间 (s) */
    WheelSpeed speed_hold_;                               /**< 窗口之间保持的转速(零阶保持) */
    /**
    * @brief 计算两个 16 位硬件计数器之间的差值，自动处理回绕（Wrap-around）
    * @param current 当前读取的 CNT 寄存器值
    * @param prev    上一次记录的 CNT 值
    * @return 有符号的 32 位差值（正数表示正转，负数表示反转）
    */
    int32_t calc_encoder_delta(uint16_t current, uint16_t prev);
};


#endif //DRIVERS_MOTOR_A4950_A4950MOTORDRIVER_H
