//
// 四轮电机驱动抽象接口
//
// 与芯片无关：本文件不允许出现任何 STM32 相关的类型、寄存器或引脚定义。
// 具体实现见 drivers/ 下的器件驱动，仿真实现可自行替换。
//

#ifndef CORE_INTERFACES_IWHEELDRIVER_HPP
#define CORE_INTERFACES_IWHEELDRIVER_HPP

#include <cstdint>

/**
 * @brief 电机/占空比相关常量
 */
namespace MotorParam {
    /**
     * @brief PWM 占空比量化上限(绝对值)，WheelPwm 各字段值域为 [-PWM_MAX, +PWM_MAX]
     * @note  放在接口层是因为它属于"硬件接口的取值约定"，控制器与驱动都要用；
     *        具体数值必须与实现侧的定时器 ARR 一致，否则占空比满量程对不上
     */
    constexpr int16_t PWM_MAX = 8399;

    /**
     * @brief 默认轮速上限
     * @unit rps (转/秒)
     * @note 取值来自电机空载最大转速，需按实车复核
     */
    constexpr float MAX_WHEEL_SPEED = 5.0f;
}

/**
 * @brief 四轮索引
 * @note  用于 wheel_pid_[kWheelLf] 这类按轮索引的场合，避免魔法下标；
 *        同时明确了"轮序 LF, RF, LR, RR"这一全局约定，
 *        该顺序与 WheelSpeed / WheelPwm 的字段顺序严格一致
 */
enum WheelIndex {
    kWheelLf = 0,    /**< 左前轮 */
    kWheelRf = 1,    /**< 右前轮 */
    kWheelLr = 2,    /**< 左后轮 */
    kWheelRr = 3,    /**< 右后轮 */
    kWheelCount = 4  /**< 轮子总数 */
};

/**
 * @brief 四轮转速
 * @note  单位是 rps(转/秒)而不是 rpm，命名与单位保持一致；
 *        字段顺序固定为 LF, RF, LR, RR，与 WheelIndex 一致
 */
struct WheelSpeed {
    float lf;  /**< 左前轮转速 (rps) */
    float rf;  /**< 右前轮转速 (rps) */
    float lr;  /**< 左后轮转速 (rps) */
    float rr;  /**< 右后轮转速 (rps) */

    /**
     * @brief 默认构造，四轮置零
     */
    WheelSpeed() : lf(0.0f), rf(0.0f), lr(0.0f), rr(0.0f) {}

    /**
     * @brief 依次构造
     */
    WheelSpeed(float lf_, float rf_, float lr_, float rr_)
        : lf(lf_), rf(rf_), lr(lr_), rr(rr_) {}
};

/**
 * @brief 四轮 PWM 占空比指令
 * @note  必须与 WheelSpeed 分开定义：PWM 是整数量化的占空比，转速是物理量，
 *        两者语义与单位都不同。共用一个类型会导致"把目标转速当 PWM 传进去"
 *        这类错误编译器完全无法发现
 */
struct WheelPwm {
    int16_t lf;  /**< 左前轮 PWM，值域 [-PWM_MAX, +PWM_MAX]，符号决定转向 */
    int16_t rf;  /**< 右前轮 PWM */
    int16_t lr;  /**< 左后轮 PWM */
    int16_t rr;  /**< 右后轮 PWM */

    /**
     * @brief 默认构造，四路输出置零
     */
    WheelPwm() : lf(0), rf(0), lr(0), rr(0) {}

    /**
     * @brief 依次构造
     */
    WheelPwm(int16_t lf_, int16_t rf_, int16_t lr_, int16_t rr_)
        : lf(lf_), rf(rf_), lr(lr_), rr(rr_) {}
};

/**
 * @brief 轮驱动硬件接口(4 路电机 + 4 路编码器)
 * @note  面向接口编程，便于用仿真实现做离线测试与回归测试
 */
class IWheelDriver {
public:
    virtual ~IWheelDriver() {}

    /**
     * @brief 初始化电机与编码器外设
     * @return true 初始化成功，false 失败
     */
    virtual bool init() = 0;

    /**
     * @brief 反初始化，释放外设资源
     */
    virtual void deinit() = 0;

    /**
     * @brief 查询是否已初始化成功
     * @return true 已初始化
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief 使能/失能电机输出
     * @param[in] enabled true 使能，false 失能
     * @note  失能后 setPwm() 不应产生实际输出
     * @note  失能**不**停止编码器：编码器是只读输入，停掉它会让 getWheelSpeed()
     *        恒返回 0，停机状态机就判不出"车还在滑行"。编码器随 init() 工作，
     *        只有 deinit() 才停
     */
    virtual void setEnabled(bool enabled) = 0;

    /**
     * @brief 查询使能状态
     * @return true 已使能
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief 输出四路 PWM
     * @param[in] pwm 四轮 PWM 指令，越界值由实现负责钳位到 [-PWM_MAX, +PWM_MAX]
     * @note  未初始化时调用无效(实现应直接返回)
     */
    virtual void setPwm(const WheelPwm& pwm) = 0;

    /**
     * @brief 读取四轮实时转速
     * @param[in] dt 距上一次调用本函数的时间间隔，单位 s，必须 > 0
     * @return 四轮转速 (rps)
     * @note  转速是编码器计数的差分，所以周期必须由调用方显式传入；
     *        dt 必须与实际两次调用之间的时间一致，且调用周期应固定。
     *        实现内部按 (Δ计数 / 单圈计数 / 减速比) / dt 计算
     * @note  实现允许在内部做多拍累计(测速窗口)以抑制量化噪声，
     *        此时窗口之内的返回值保持不变，调用方不必感知
     */
    virtual WheelSpeed getWheelSpeed(float dt) = 0;

    /**
     * @brief 滑行停机
     * @note 会导致驱动芯片休眠，再次启动出现延时
     * @warning 执行时会关闭 PWM 定时器，应在再启动时调用恢复
     */
    virtual void CoastStop_WithStartupDelay_PWMDisabled() = 0;

    /**
     * @brief 主动停机(绕组短接制动)
     * @warning 执行时会关闭 PWM 定时器，应在再启动时调用恢复
     */
    virtual void BrakeStop() = 0;

    /**
     * @brief 把 PWM 引脚恢复成定时器复用功能并重启输出
     */
    virtual void RestorePWM_GPIO() = 0;
};

#endif // CORE_INTERFACES_IWHEELDRIVER_HPP
