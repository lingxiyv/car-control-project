//
// 角度与数值基础运算
//
// 与芯片无关的纯数学工具。放在 core 是因为控制层与未来的滤波/解算都要用，
// 且它本身不含任何硬件依赖。
//

#ifndef CORE_MATH_ANGLE_HPP
#define CORE_MATH_ANGLE_HPP

#include <cmath>

namespace core_math {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

/**
 * @brief 把角度归一化到 [-π, π)
 * @param[in] angle 任意角度，单位 rad
 * @return 等价角度，落在 [-π, π)
 * @note  用于把航向误差压到"最短路径"上：航向环必须基于归一化后的误差，
 *        否则 179° 与 -179° 之间 2° 的小偏差会被算成 358° 的大偏差
 */
inline float normalizeAngle(float angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0f) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

/**
 * @brief 把数值钳位到 [min_value, max_value]
 */
inline float clampFloat(float value, float min_value, float max_value) {
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

} // namespace core_math

#endif // CORE_MATH_ANGLE_HPP
