//
// Created by lingxi on 2026/10/14.
//


#ifndef A_TEST_PID_H
#define A_TEST_PID_H

/**
 *@brief    通用PID控制类
 *          纯算法模块，无硬件依赖，用于速度环、角速度环的闭环计算
 *          位置式 PID：out = kp*e + ki*∫e·dt + kd*de/dt
 *          微分项对误差求导，首拍跳过微分以避免微分冲击
 *          抗积分饱和：积分项按 setIntegralLimit() 限幅，
 *                      且输出达到限幅时暂停积分累加(条件积分)
 *          建议输出量纲归一为 0.0~1.0 或 -1.0~1.0 （根据输出限幅）
 *@note     可用有参构造填入参数，也可先默认构造再用 setParameters() 配置；
 *          后者是为了能作为数组/成员直接定义
 *          (例如 ChassisController 中的四路速度环 PID wheel_pid_[4])
 *@note     calculate() 必须按固定周期调用，dt 由调用方以秒为单位显式传入，
 *          这样 ki/kd 才有明确的时间量纲，修改控制周期不需要重调整套参数
 *@author   泠熙语云
 *@date     2026-10-14
 *@version  1.1
 */
class PID {
    public:
    /**
     * @brief 默认构造：kp/ki/kd 均为 0，输出恒为 0，需再调用 setParameters() 配置
     * @note  输出限幅默认 ±1.0(与有参构造一致)，积分限幅默认不限制；
     *        实际使用前仍应显式调用 setOutputLimit()/setIntegralLimit()
     */
    PID();

    /**
     * @brief 初始化PID参数
     * @param kp 比例
     * @param ki 积分
     * @param kd 微分
     * @note  输出限幅与积分限幅仍需另外调用 setOutputLimit()/setIntegralLimit()
     */
    PID(float kp, float ki, float kd);

    /**
     * @brief 设置输出限幅
     * @param min 输出下限
     * @param max 输出上限，应满足 max >= min
     * @note  calculate() 的返回值会被钳位到 [min, max]
     */
    void setOutputLimit(float min, float max);

    /**
     * @brief 设置积分项限幅(抗积分饱和)
     * @param limit 积分累加项的绝对值上限
     * @note  limit <= 0 表示不限制(不建议)
     */
    void setIntegralLimit(float limit);

    /**
     * @brief 执行一次PID计算
     * @param[in] setpoint 目标值
     * @param[in] feedback 反馈测量值
     * @param[in] dt 距上一次调用 calculate() 的时间间隔，单位 s，必须 > 0
     * @return PID输出结果，已经过输出限幅
     * @note  非线程安全，同一实例不可在中断与主循环中同时调用；
     *        反馈数据异常(如编码器读数为 0 或明显跳变)的判定由调用方负责
     */
    float calculate(float setpoint, float feedback, float dt);

    /**
     * @brief 重置PID状态：积分项、历史误差清零
     * @note  切换控制模式、重新使能、被控对象状态突变后应调用，
     *        否则会带着旧的积分量产生输出突变
     */
    void reset();

    /**
     * @brief 设置PID参数，仅调试
     * @param kp 比例
     * @param ki 积分
     * @param kd 微分
     */
    void setParameters(float kp, float ki, float kd);

    /**
     * @brief 读取当前比例系数
     * @return kp
     */
    float getKp() const;

    /**
     * @brief 读取当前积分系数
     * @return ki
     */
    float getKi() const;

    /**
     * @brief 读取当前微分系数
     * @return kd
     */
    float getKd() const;

    /**
     * @brief 读取当前积分累加项
     * @return 积分项当前值，用于调试与上位机遥测
     */
    float getIntegral() const;

private:
    float kp_;              /**< 比例系数 */
    float ki_;              /**< 积分系数 */
    float kd_;              /**< 微分系数 */
    float integral_;        /**< 积分累加项 */
    float prev_error_;      /**< 上一次误差，用于计算微分项 */
    bool first_run_;        /**< 首拍标志，首拍跳过微分计算 */
    float out_min_;         /**< 输出下限 */
    float out_max_;         /**< 输出上限 */
    float integral_limit_;  /**< 积分项绝对值上限，<=0 表示不限制 */
};


#endif //A_TEST_PID_H
