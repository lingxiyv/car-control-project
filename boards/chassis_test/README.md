# 硬件配置组合：chassis_test

`boards/` 下**一个目录 = 一套硬件配置组合**——引脚映射 + 外设实例化 + 时钟树 +
内存/链接布局，由本目录的 `.ioc` 描述、由构建期变量 `BOARD` 选中。
它**不是**"一块物理板"的抽象。

判断要不要新建目录，只看一条：

> **要不要动 `BoardPins.h` / `.ioc` / 链接脚本？要动 → 新建组合；
> 只是宏或编译选项不同 → 用 preset 或 CMake 缓存变量。**

几个容易走偏的用法：

- 电气等价的两块 PCB 用**同一个组合**即可，不必因为"换了块板子"就新建目录。
- 同一块 PCB 只要接线或外设用途不同，就是**两个不同的组合**。
- 把"这个构建目标想用 CAN 而不是 UART"这类**纯构建配置差异**也做成新组合，
  `boards/` 就会退化成第二个 `bsp/`，失去隔离意义。

本目录对应 `BOARD=chassis_test` 这一组合：
- 板卡：自绘麦轮底盘板，A4950 ×4 + 增量编码器 ×4
- 引脚分配：见 `BoardPins.h` / `chassis_test.ioc`
- 若引脚功能或接线变化，新建 `boards/<new_combo>/`，不要修改本目录

---

## 版本记录

| 项 | 值 |
|---|---|
| CubeMX | 6.18.1（MxDb `DB.6.0.181`） |
| STM32Cube FW_F4 HAL | 1.8.5（`third_party/stm32/STM32F4xx_HAL_Driver`） |
| MCU | STM32F407xx，168 MHz |
| 板卡 | 自绘麦轮底盘板，A4950 ×4 + 增量编码器 ×4 |

> 重新生成后请核对上表。CubeMX 会带**它自己那份** HAL；如果与
> `third_party/stm32/` 里的版本不一致，**没有任何工具会警告你**，
> 只会在 `cmake/board.cmake` 硬编码的 HAL 源文件列表上悄悄失败或改变行为。

---

## 哪些能改

**手写（可以改）**

| 文件 | 说明 |
|---|---|
| `BoardPins.h` | 唯一的板级对外契约。引脚语义化都在这，`drivers/`、`apps/` 只许 include 它 |
| `chassis_test.ioc` | CubeMX 工程文件。**视作二进制**：不要在同一配置组合的两个分支上同时改 |

**生成（改了会被覆盖）**

| 路径 | 说明 |
|---|---|
| `Core/Inc/`、`Core/Src/` | `main.c` / `tim.c` / `gpio.c` / `usart.c` / `i2c.c` / 中断 / MSP / sysmem / syscalls / system_stm32f4xx.c |
| `startup_stm32f407xx.s` | 启动文件 |
| `STM32F407xx_FLASH.ld` | 链接脚本（由顶层 CMakeLists 按 `${BOARD}` 传入 `-T`） |

---

## CubeMX 工程设置（关键）

在 `.ioc` 的 **Project Manager** 里必须满足：

1. **Project Path 指向本目录**（`boards/chassis_test/`），不是仓库根。
2. **关闭 "Generate Under Root"**。

否则 CubeMX 会去覆盖仓库根目录的 `CMakeLists.txt` 和 `CMakePresets.json`
——那两份是手写维护的构建入口，一旦被覆盖，把 `.ioc` 放进 `boards/` 就白隔离了。

---

## 每次重新生成后的固定动作

CubeMX 会在本目录额外生成一套完整的 CMake 工程（`CMakeLists.txt`、`cmake/`、`Drivers/`、可能还有
`Middlewares/`）。本工程的构建脚本在仓库根目录手写维护，这些一律不用：

```bat
rem 在仓库根目录执行
rmdir /S /Q boards\chassis_test\cmake
rmdir /S /Q boards\chassis_test\Drivers
rmdir /S /Q boards\chassis_test\Middlewares
del /Q      boards\chassis_test\CMakeLists.txt
```

`Drivers/` 是 CubeMX 给每块板复制的 HAL（约 100 个文件）。本工程只保留
`third_party/stm32/` 下唯一一份共享副本。**删之前先 diff 一下版本**：

```bat
fc /B third_party\stm32\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal.h ^
      boards\chassis_test\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal.h
```

若版本不同，要么升级 `third_party/stm32/`，要么把 `.ioc` 里的固件包版本降回一致——
不要让两者长期分叉。

`Core/`、启动文件、链接脚本**不需要搬动**，`cmake/board.cmake` 已经按
`boards/<board>/Core/...` 列出。

`.gitignore` 已忽略上述生成物，`git status` 里不应该出现它们。
