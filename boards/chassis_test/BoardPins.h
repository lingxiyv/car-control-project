//
// 板级引脚映射：电机驱动与编码器
//
// 本文件是**板级唯一的对外契约**：drivers/ 与 apps/ 只允许 include 它，
// 不允许直接 include Core/Inc/tim.h、main.h、gpio.h。
// 守住这条，换板子时只需要换这一份头文件。
//
// 它把 CubeMX 生成的外设句柄与引脚宏统一收口到这里。
//

#ifndef BOARD_CHASSIS_TEST_BOARDPINS_H
#define BOARD_CHASSIS_TEST_BOARDPINS_H

// 提供 HAL 类型、GPIO 端口宏，以及 htim1/htim2/htim3/htim4/htim8 句柄。
// tim.h 内部已经 include 了 main.h。
#include "tim.h"

// ---- 4 路 A4950 控制引脚 ----
// 每路：IN1 为普通 GPIO(静态定方向)，IN2 为定时器通道(承载 PWM)。
// 具体原因见 A4950MotorDriver.cpp 中 applyWheelOutput() 的注释。
#define MOTOR_LF_IN1_PIN    GPIO_PIN_0
#define MOTOR_LF_IN1_PORT   GPIOF
#define MOTOR_LF_IN2_PIN    GPIO_PIN_8
#define MOTOR_LF_IN2_PORT   GPIOA

#define MOTOR_RF_IN1_PIN    GPIO_PIN_1
#define MOTOR_RF_IN1_PORT   GPIOF
#define MOTOR_RF_IN2_PIN    GPIO_PIN_9
#define MOTOR_RF_IN2_PORT   GPIOA

#define MOTOR_LR_IN1_PIN    GPIO_PIN_2
#define MOTOR_LR_IN1_PORT   GPIOF
#define MOTOR_LR_IN2_PIN    GPIO_PIN_10
#define MOTOR_LR_IN2_PORT   GPIOA

#define MOTOR_RR_IN1_PIN    GPIO_PIN_3
#define MOTOR_RR_IN1_PORT   GPIOF
#define MOTOR_RR_IN2_PIN    GPIO_PIN_11
#define MOTOR_RR_IN2_PORT   GPIOA

// ---- 4 路编码器对应的定时器 ----
#define ENCODER_LF TIM2
#define ENCODER_RF TIM3
#define ENCODER_LR TIM4
#define ENCODER_RR TIM8

#endif // BOARD_CHASSIS_TEST_BOARDPINS_H
