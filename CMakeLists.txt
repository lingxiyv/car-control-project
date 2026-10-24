cmake_minimum_required(VERSION 3.22)

# =============================================================================
# 顶层构建脚本（手写维护）
#
# CubeMX 的 .ioc 放在 boards/<board>/ 下，因此 CubeMX 重新生成时只会改写
# boards/<board>/ 内的文件，不会覆盖本文件与 cmake/ 下的脚本。
# =============================================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME chassis_test)

# 便于 clangd 等工具索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})

enable_language(C ASM)

# -----------------------------------------------------------------------------
# 选择硬件配置组合
#   cmake --preset Debug -DBOARD=chassis_v2
# 每个组合一个目录：boards/<board>/，内含该组合的 .ioc 与 CubeMX 生成的
# 引脚/时钟初始化。不同目标改引脚只需要换 BOARD，不需要改任何源码。
# -----------------------------------------------------------------------------
set(BOARD "chassis_test" CACHE STRING "硬件配置组合名，对应 boards/<BOARD>/")
set(BOARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}")

if(NOT EXISTS "${BOARD_DIR}/Core/Src/main.c")
    file(GLOB _available RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/boards"
         "${CMAKE_CURRENT_SOURCE_DIR}/boards/*")
    list(JOIN _available ", " _available_str)
    message(FATAL_ERROR
        "找不到板级目录 boards/${BOARD}/ （缺少 Core/Src/main.c）。\n"
        "可用组合: ${_available_str}\n"
        "用 -DBOARD=<组合名> 选择。")
endif()
message(STATUS "Board      : ${BOARD}")
message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")

# -----------------------------------------------------------------------------
# 手写代码
# -----------------------------------------------------------------------------
add_executable(${CMAKE_PROJECT_NAME}
    # core —— 与芯片无关的算法与接口
    core/pid/PID.cpp

    # drivers —— 器件驱动（绑器件类型，不绑板子）
        drivers/motor/A4950/A4950MotorDriver.cpp

    # apps —— 具体业务
    apps/app_mecanum_test/ChassisController.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    core/interfaces
    core/pid
    core/math

        drivers/motor

    apps/app_mecanum_test

    # 板级：BoardPins.h 在板目录根下，main.h / tim.h 等在 Core/Inc 下
    ${BOARD_DIR}
    ${BOARD_DIR}/Core/Inc
)

# 板级源文件 + 厂商 HAL + 板级头文件路径/宏定义
include(cmake/board.cmake)

# 链接脚本随配置组合走（不写死在工具链里）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T "${BOARD_DIR}/STM32F407xx_FLASH.ld"
)

# 使用 C++ 源文件时移除错误的 libob.a 隐式依赖
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME} m)
