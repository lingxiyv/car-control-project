# -----------------------------------------------------------------------------
# 板级 + 厂商 HAL 的源文件、头文件路径与宏定义
#
# 由顶层 CMakeLists.txt include（include 不切换目录作用域，
# 因此这里可以继续用 ${CMAKE_CURRENT_SOURCE_DIR} 指仓库根）。
#
# 依赖父级定义的变量：
#   BOARD      硬件配置组合名
#   BOARD_DIR  boards/${BOARD} 的绝对路径
#
# 注意：这里**不**使用 boards/<board>/cmake/ 下 CubeMX 生成的那套脚本。
#   原因：它会把每块板自己的 Drivers/ 也一起编进去，而本工程只共享
#   third_party/stm32 下的一份 HAL。CubeMX 每次生成后把
#   boards/<board>/CMakeLists.txt 与 boards/<board>/cmake/ 删掉即可。
# -----------------------------------------------------------------------------

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/stm32")

# ---- 板级源文件（CubeMX 生成，随配置组合不同而变化） ----
set(BOARD_Src
    ${BOARD_DIR}/Core/Src/main.c
    ${BOARD_DIR}/Core/Src/gpio.c
    ${BOARD_DIR}/Core/Src/i2c.c
    ${BOARD_DIR}/Core/Src/tim.c
    ${BOARD_DIR}/Core/Src/usart.c
    ${BOARD_DIR}/Core/Src/stm32f4xx_it.c
    ${BOARD_DIR}/Core/Src/stm32f4xx_hal_msp.c
    ${BOARD_DIR}/Core/Src/sysmem.c
    ${BOARD_DIR}/Core/Src/syscalls.c
    ${BOARD_DIR}/Core/Src/system_stm32f4xx.c
    ${BOARD_DIR}/startup_stm32f407xx.s
)

# ---- 厂商 HAL 源文件（全工程共享一份，不随配置组合复制） ----
set(HAL_Src
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${BOARD_Src} ${HAL_Src})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Inc
    ${THIRD_PARTY_DIR}/STM32F4xx_HAL_Driver/Inc/Legacy
    ${THIRD_PARTY_DIR}/CMSIS/Device/ST/STM32F4xx/Include
    ${THIRD_PARTY_DIR}/CMSIS/Include
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)
