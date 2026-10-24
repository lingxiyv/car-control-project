//
// 航向传感器抽象接口
//
// 与芯片无关：本文件不允许出现任何 STM32 相关的类型、寄存器或引脚定义。
// 具体实现(IMU 驱动)由各自的驱动类提供。
//

#ifndef CORE_INTERFACES_IHEADINGSENSOR_HPP
#define CORE_INTERFACES_IHEADINGSENSOR_HPP

/**
 * @brief 航向传感器接口
 * @note  单独成接口而不是塞进 IWheelDriver：航向传感器是底盘级设备，
 *        与轮驱动没有必然绑定关系；拆开后仿真与真实硬件可以各自替换。
 *        角度与角速度统一为逆时针为正(右手系，与 REP-103 一致)
 */
class IHeadingSensor {
public:
    virtual ~IHeadingSensor() {}

    /**
     * @brief 初始化传感器外设
     * @return true 初始化成功，false 失败
     */
    virtual bool init() = 0;

    /**
     * @brief 查询是否已初始化成功
     * @return true 已初始化
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief 读取航向角
     * @return 航向角，单位 rad，逆时针为正
     */
    virtual float getYaw() = 0;

    /**
     * @brief 读取航向角速度
     * @return 角速度，单位 rad/s，逆时针为正
     * @note  供角速度环做反馈用；不需要时可忽略
     */
    virtual float getYawRate() = 0;

    /**
     * @brief 将当前航向置为 0
     * @note  重新标定时调用
     */
    virtual void resetYaw() = 0;

    /**
     * @brief 设置静止零偏
     * @param offset 偏移量，单位 rad/s
     * @return 实际生效的零偏
     */
    virtual float setStaticOffset(float offset) = 0;

    /**
     * @brief 查询是否已完成校准
     * @return true 已校准
     */
    virtual bool isCalibrated() const = 0;
};

#endif // CORE_INTERFACES_IHEADINGSENSOR_HPP
