//
// Created by lingxi on 2026/10/14.
//

#include "PID.h"


PID::PID() {
    kp_ = 0.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_run_ = true;
    out_min_ = -1.0f;
    out_max_ = 1.0f;
    integral_limit_ = 0.0f;
}

PID::PID(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_run_ = true;
    //与默认构造保持一致：默认限幅为 ±1.0 而不是 [0, 0]。
    //写成 [0, 0] 会让用三参构造出来的 PID 静默地恒输出 0，极难排查
    out_min_ = -1.0f;
    out_max_ = 1.0f;
    integral_limit_ = 0.0f;
}

void PID::setOutputLimit(float min, float max) {
    out_min_ = min;
    out_max_ = max;
}

void PID::setIntegralLimit(float limit) {
    integral_limit_ = limit;
}

float PID::calculate(float setpoint, float feedback, float dt) {
    //防御性检查：防止除零或负时间
    if (dt <= 0.0f) return 0.0f;

    //计算误差
    const float error = setpoint - feedback;

    //计算微分项：首拍跳过，避免微分冲击
    float derivative = 0.0f;
    if (first_run_) {
        first_run_ = false;
    } else {
        derivative = (error - prev_error_) / dt;
    }

    const float p_term = kp_ * error;
    const float d_term = kd_ * derivative;

    //条件积分(抗积分饱和)：先看"用当前积分算出来的输出"是不是已经顶在限幅上、
    //且误差还在往同一个方向推，如果是就冻结积分。
    //不做"先预测再判断"：那样会在输出还差一点没饱和时就提前冻结积分，
    //留下一个靠积分本该能消掉、却永远消不掉的静差。
    const float out_before = p_term + ki_ * integral_ + d_term;
    const bool push_further =
        (out_before >= out_max_ && error > 0.0f) ||
        (out_before <= out_min_ && error < 0.0f);

    if (!push_further) {
        integral_ += error * dt;
        //积分硬限幅：作为兜底，防止积分量本身无限增长
        if (integral_limit_ > 0.0f) {
            if (integral_ > integral_limit_) {
                integral_ = integral_limit_;
            } else if (integral_ < -integral_limit_) {
                integral_ = -integral_limit_;
            }
        }
    }

    //计算 PID 输出并限幅
    float pid_value = p_term + ki_ * integral_ + d_term;
    if (pid_value > out_max_) {
        pid_value = out_max_;
    } else if (pid_value < out_min_) {
        pid_value = out_min_;
    }

    //更新状态
    prev_error_ = error;
    return pid_value;
}

void PID::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_run_ = true;
}

void PID::setParameters(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float PID::getKd() const {
    return kd_;
}

float PID::getKp() const {
    return kp_;
}

float PID::getKi() const {
    return ki_;
}

float PID::getIntegral() const {
    return integral_;
}
