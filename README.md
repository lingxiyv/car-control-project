# 麦克纳姆轮底盘控制固件

面向 **STM32F407 + A4950** 的四轮麦克纳姆轮底盘控制固件，C++ 实现。

四路速度环 + 单路角速度环构成双闭环；上层以**世界坐标系**下的航迹指令驱动底盘，
世界系→车体系的变换与运动学解算都在固件内部完成。

> **项目状态**：核心控制层（运动学、双闭环、停机状态机）已实现且可编译；
> 板级初始化与控制器尚未在主循环/控制中断里串起来，IMU 驱动也还没有落地。
> 详见 [已知限制](#已知限制)。

---

## 项目简介

- **双闭环**：四个轮子各一个速度环 PID，外加一个共用的角速度环 PID 负责车体朝向。
- **世界系航迹**：上层只说"往世界系的哪个方向、以多快走"，车体转过多少度都不影响指令含义。
- **角速度独占**：航迹方向修正与自旋指令**不叠加**，用 `AngularControl` 显式表达归属，
  避免两个目标互相打架（早期版本的真实 bug）。
- **分层解耦**：控制器只依赖 `IWheelDriver` / `IHeadingSensor` 两个纯虚接口，
  不含任何寄存器操作，换掉实现即可脱离硬件编译
  （注：`apps/` 目前仍直接调用 HAL 的 `HAL_Delay` / `HAL_GetTick`，
  要真在 PC 上跑仿真得先把这几处抽走，见[已知限制](#已知限制)）。
- **换板零改源码**：引脚、外设、时钟、链接脚本全部收在 `boards/<配置组合>/` 下，
  靠构建期变量 `BOARD` 切换。

---

## 目标硬件

| 项目 | 配置 |
|---|---|
| MCU | STM32F407ZGTx（Cortex-M4F，168 MHz，1 MB Flash / 192 KB SRAM，含 64 KB CCM） |
| 电机驱动 | **A4950 ×4**，IN/IN 模式，无 EN 脚 |
| 电机 | CDR-HM37-520 减速电机（30:1），自带增量式编码器 |
| 编码器 | 11 线，`TI12` 四倍频，轮轴每转 1320 计数 |
| 航向传感器 | I2C2 上的 IMU，100 kHz |
| 调试串口 | USART2，9600 |

| 信号 | 引脚 / 外设 |
|---|---|
| PWM 输出 | `TIM1_CH1~CH4` → `PA8 / PA9 / PA10 / PA11`（20 kHz，`ARR = 8399`） |
| 方向脚 IN1 | LF=`PF0`　RF=`PF1`　LR=`PF2`　RR=`PF3` |
| 编码器 | LF=`TIM2`　RF=`TIM3`　LR=`TIM4`　RR=`TIM8`（16 位，`TI12` 四倍频） |

机构参数见 `ChassisParam`：轮半径 0.065 m、轮距 0.22 m、轴距 0.20 m。
引脚映射见 [`boards/chassis_test/BoardPins.h`](boards/chassis_test/BoardPins.h)。

> **A4950 注意**：IN1 只能是静态电平，PWM 只能加在 IN2 上。这使得正转是"刹车调制"
> （平均电压 ∝ `1-d`）、反转是"滑行调制"（∝ `d`），两个方向极性相反。
> `applyWheelOutput()` 里对正转写了补码 `CCR = PWM_MAX - duty` 做补偿，**不要"简化"掉这一步**，
> 否则正向闭环增益为负，PID 会把轮子锁死在满刹车。

---

## 仓库结构

四层，依赖只能自上而下：

```
core/                     与芯片无关的核心资产，换 MCU/换板子都不用改
  interfaces/             IWheelDriver.hpp / IHeadingSensor.hpp 纯虚接口 + 共享数据类型
  pid/                    PID 算法（位置式，微分首拍跳过 + 条件积分抗饱和）
  math/                   角度归一化、数值钳位

drivers/                  器件驱动：绑定"器件类型"，不绑定具体板子
  motor/A4950/            A4950MotorDriver.* —— A4950 + 增量编码器的驱动实现

boards/                   硬件配置组合：一个目录 = 一套自洽的引脚/时钟/外设/内存布局
  chassis_test/
    chassis_test.ioc      CubeMX 工程文件
    BoardPins.h           该组合的引脚映射（手写，换接线只改这里）
    Core/Src, Core/Inc    CubeMX 生成的 main.c / tim.c / gpio.c 等
    startup_*.s          该组合的启动文件
    STM32F407xx_FLASH.ld 该组合的链接脚本
    README.md             该组合的维护细则（CubeMX 设置、重新生成后的清理）

apps/                     具体业务
  app_mecanum_test/       ChassisController.* —— 运动学解算、双闭环、停机状态机

third_party/
  stm32/                  STM32 HAL 与 CMSIS（vendored，全工程共享一份，不要手改）

cmake/
  board.cmake             板级 + HAL 的源文件列表、头文件路径、宏定义
  gcc-arm-none-eabi.cmake GCC 工具链文件（默认）
  starm-clang.cmake       clang 工具链文件（可选）

CMakeLists.txt            顶层构建脚本（手写维护，CubeMX 不会碰）
CMakePresets.json         Debug / Release 两套 preset
```

| 文件 | 职责 |
|---|---|
| [`core/interfaces/IWheelDriver.hpp`](core/interfaces/IWheelDriver.hpp) | 四轮电机接口、`WheelSpeed`/`WheelPwm`、`MotorParam` 常量 |
| [`core/interfaces/IHeadingSensor.hpp`](core/interfaces/IHeadingSensor.hpp) | 航向传感器接口 |
| [`core/pid/PID.h`](core/pid/PID.h) / [`PID.cpp`](core/pid/PID.cpp) | 通用位置式 PID |
| [`core/math/Angle.hpp`](core/math/Angle.hpp) | 角度归一化、钳位 |
| [`drivers/motor/A4950/A4950MotorDriver.h`](drivers/motor/A4950/A4950MotorDriver.h) | PWM 输出（含 A4950 极性补偿）、编码器测速、滑行/急刹 |
| [`boards/chassis_test/BoardPins.h`](boards/chassis_test/BoardPins.h) | 该配置组合的引脚映射（板级唯一对外契约） |
| [`apps/app_mecanum_test/ChassisController.h`](apps/app_mecanum_test/ChassisController.h) | 控制层：运动学解算、双闭环、速度快照、停机状态机 |

### 关于 `boards/`：它是"硬件配置组合"，不是"物理板"

`boards/` 下**一个目录 = 一套硬件配置组合**——引脚映射 + 外设实例化 + 时钟树 +
内存/链接布局，由 `.ioc` 描述、由构建期变量 `BOARD` 选中。

它**不是**"仓库里存了几块板子"的意思，也不该按"换了一块 PCB 就新建一个目录"来维护：

- 一个组合可以由**拼装/飞线**任意实现，只要它的引脚与电气特性自洽，就能编译并烧录。
- 同一块 PCB 只要接线或外设用途不同，就是**两个不同的组合**。
- 电气等价的两块 PCB，用**同一个组合**即可，不必新增目录。
- "这个构建目标想用 CAN 而不是 UART"这类**纯构建配置差异**，不该做成新组合，
  否则 `boards/` 会退化成第二个 `bsp/`，失去隔离意义。

判定标准只有一条：**要不要动 `BoardPins.h` / `.ioc` / 链接脚本**。要动 → 新组合；
只是宏或编译选项不同 → 用 preset 或 CMake 缓存变量。

选中与扩展：

```bash
cmake --preset Debug -DBOARD=chassis_v2      # 选另一个配置组合
```

新增组合的步骤：复制 `boards/chassis_test/` → 改名 → 用 CubeMX 按新目标改 `.ioc` → 生成。
`CMakePresets.json` 里 `BOARD` 的默认值决定当前分支用哪个组合。
更细的维护规则见 [`boards/chassis_test/README.md`](boards/chassis_test/README.md)。

---

## 构建

### 依赖

| 工具 | 版本要求 | 实测版本 |
|---|---|---|
| CMake | ≥ 3.22 | 4.3.1 |
| Ninja | 任意 | 1.13.2 |
| `arm-none-eabi-gcc` | 支持 `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16` | GNU Tools for STM32 14.3.rel1（GCC 14.3.1） |

`arm-none-eabi-` 系列工具必须在 `PATH` 里（工具链文件按前缀查找，不做绝对路径绑定）。

### clone 即编译

**`third_party/` 是 vendored 的**——HAL 与 CMSIS 的源码直接躺在仓库里，
不带 submodule、不需要联网拉依赖、不需要先装 STM32Cube 包：

```bash
git clone <your-repo-url> chassis_test
cd chassis_test

cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/chassis_test.elf`（以及同目录的 `.map`，链接时会打印内存占用）。

仓库不绑定烧录方式，用自己手边的工具即可：

- STM32CubeCLT 自带（本机实测可用）：
  `STM32_Programmer_CLI -c port=SWD -w build/Debug/chassis_test.elf -v -rst`
- OpenOCD：
  `openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Debug/chassis_test.elf verify reset exit"`

### CMake preset

[`CMakePresets.json`](CMakePresets.json) 有两套 preset，都继承自隐藏的 `default`：

| preset | `CMAKE_BUILD_TYPE` | 优化 | 产物目录 |
|---|---|---|---|
| `Debug` | `Debug` | `-O0 -g3` | `build/Debug/` |
| `Release` | `Release` | `-Os -g0` | `build/Release/` |

```bash
cmake --preset Release
cmake --build --preset Release
```

`default` 里固定了三件事：生成器 Ninja、`binaryDir = build/${presetName}`、
工具链文件 `cmake/gcc-arm-none-eabi.cmake`。

### 工具链

| 文件 | `TOOLCHAIN_PREFIX` | 说明 |
|---|---|---|
| [`cmake/gcc-arm-none-eabi.cmake`](cmake/gcc-arm-none-eabi.cmake) | `arm-none-eabi-` | **默认**，所有 preset 使用它；`--specs=nano.specs` + newlib-nano |
| [`cmake/starm-clang.cmake`](cmake/starm-clang.cmake) | `starm-` | 可选，STARM clang 工具链；默认 `STARM_PICOLIBC` 配置 |

换工具链不走 preset，直接覆盖缓存变量重新 configure：

```bash
cmake --preset Debug -DCMAKE_TOOLCHAIN_FILE=cmake/starm-clang.cmake
cmake --build --preset Debug
```

`starm-clang.cmake` 需要自己准备好 `starm-clang` 工具链并放进 `PATH`；
若选 `STARM_HYBRID` 配置，还要额外设置 `CLANG_GCC_CMSIS_COMPILER` 与 `GCC_TOOLCHAIN_ROOT` 两个环境变量。

> **状态**：clang 通路**尚未实测**（作者手边只有 GCC）。两份工具链文件都不再写死链接脚本，
> `-T` 统一由顶层 `CMakeLists.txt` 按 `${BOARD}` 追加。

链接脚本由顶层 `CMakeLists.txt` 按 `${BOARD}` 传入，**不写死在工具链文件里**——
不同配置组合的 FLASH/RAM 布局可能不同，工具链文件本身应当与板子无关。

### CubeMX 版本与重新生成

| 项 | 值 |
|---|---|
| STM32CubeMX | **6.18.1**（MxDb `DB.6.0.181`） |
| 固件包 | STM32Cube FW_F4 **V1.28.3** |
| HAL | STM32F4xx HAL **1.8.5**（仓库内 `third_party/stm32/`） |
| 目标 MCU | `STM32F407ZGTx` |
| Project Manager → Toolchain | `CMake` |

`.ioc` 放在 `boards/<组合>/` 内，所以 CubeMX 会把产物直接写到该目录，
**不会污染仓库根目录**（前提是 Project Manager 里关掉 "Generate Under Root"，
详见该目录的 README）。

CubeMX 每次生成会额外吐出几样本工程用不上的东西，删掉即可：

```bat
rmdir /S /Q boards\<board>\cmake
rmdir /S /Q boards\<board>\Drivers
rmdir /S /Q boards\<board>\Middlewares
del /Q      boards\<board>\CMakeLists.txt
```

- `Drivers/`：CubeMX 会给每块板复制一份 HAL。本工程只保留 `third_party/stm32/` 下的一份共享副本，
  [`cmake/board.cmake`](cmake/board.cmake) 已经指向它。
- `cmake/` 与 `CMakeLists.txt`：CubeMX 生成的构建脚本，与根目录的手写脚本冲突，不用。

这些生成物已经在 `.gitignore` 里，`git status` 不应出现它们。
**重新生成后请核对上表的 HAL 版本**：CubeMX 会带它自己那份 HAL，与 `third_party/stm32/`
不一致时没有任何工具会警告你，只会在 `cmake/board.cmake` 手写的 HAL 源文件列表上悄悄失败。

---

## 控制架构

两个通道，**互不叠加**：

```
                       ┌──────────────────────────────────────────┐
   setCourseTarget ────┤ 平移通道                                  │
   setWorldTranslate   │  世界系 (vx, vy) ──按 yaw 旋转──► 车体系    │
                       └──────────────────┬───────────────────────┘
                                          │
   AngularControl 独占 ────────────────────┤
     kCourse : 航迹方向误差 ─┐               ▼
     kSpin   : 角速度误差  ──┼──► 角速度环 ──► omega ──► 逆运动学
     kIdle   : 0           ─┘                             │
                                                          ▼
                                          四路速度环 PID ──► PWM ──► 电机
                                                          ▲
                                      编码器 ──► 正运动学 ─┘ ──► 速度快照
```

**为什么平移要走世界系**：麦轮逆运动学只在**车体系**下成立（轮子固连在车体上）。
若把世界系 `(vx, vy)` 直接送进逆运动学，车体转过 90° 之后仍会沿"车头方向"走，
所谓"世界系航迹"就是假的。`update()` 每周期按当前 yaw 做一次旋转。

**为什么角速度必须独占**：`omega` 这条通道同一时刻只允许一方下指令。
早期版本把"航迹方向修正量"与"自旋指令"相加，结果是一旦下发自旋，
航迹环就把旋转拉回去，两个目标永远互相打架。现在由 `AngularControl` 表达归属：

| 接口 | 作用 | 是否夺取角速度环 |
|---|---|---|
| `setCourseTarget(course, speed)` | 世界系航迹：方向 + 速度大小 | ✅ → `kCourse` |
| `setSpinTarget(omega)` | 自旋角速度 | ✅ → `kSpin` |
| `releaseAngularControl()` | 归还控制权 | → `kIdle` |
| `getAngularControl()` | 查询当前持有者 | — |
| `setWorldTranslateTarget(vx, vy)` | 只改平移 | ❌ 不动控制权 |

夺取是"后来者胜出"，被顶掉的一方立即失效。`kSpin` 期间**平移目标保留**，
因此可以"一边自旋、一边沿固定的世界方向平移"。

---

## 坐标系与单位约定

| 量 | 约定 |
|---|---|
| 坐标系 | x 向前、y 向左、z 向上，右手系 |
| `course` / `yaw` | 世界系下的航向角，逆时针为正；0 = `init()`/`calibrationIMU()` 时刻的车头朝向 |
| `vx, vy` | m/s，前 / 左为正 |
| `omega` | rad/s，逆时针（俯视）为正 |
| 轮速 | **rps = 转/秒**（不是 rpm） |

> 命名约定：本工程 `_rps` 一律是**转/秒**，角速度一律用 `_radps`。
> 两者都是"每秒"，混用会让 2π 这个因子凭空消失。

---

## 快速上手

```cpp
A4950MotorDriver wheel;    // 驱动实现（drivers/motor/A4950/）
ImuDriver       imu;      // 航向传感器实现：仓库里还没有，需按手头 IMU 型号自行实现 IHeadingSensor
ChassisController chassis(wheel, imu);

chassis.init();                 // 初始化外设 + PID 限幅 + 世界系基准
chassis.calibrationIMU();       // 静止标定；此刻车头方向 = 世界系 +x
chassis.setEnabled(true);

// 朝世界系 30°、以 1 m/s 前进，同时把车头对到 30°
chassis.setCourseTarget(30.0f * 3.14159f / 180.0f, 1.0f);

// 切自旋：立刻夺取角速度环，平移目标保留
chassis.setSpinTarget(1.5f);    // rad/s

// 只调平移，保持自旋
chassis.setWorldTranslateTarget(0.5f, 0.0f);

chassis.releaseAngularControl();// 停止转向，平移继续
```

**调用约定**

- 控制周期入口里 `update(dt)` 与 `stop(dt)` **二选一**，不要同时调（否则速度环与刹车抢输出）
- `dt` 必须与实际调用间隔一致，且周期固定
- `setCourseTarget()` 可以每个周期重复调用（例如持续下发摇杆指令）：
  模式没变时只更新目标，不会 `reset()` PID，积分量不会被打断
- 刹车流程（`kCoasting` / `kBraking`）会自己读取轮速反馈，不依赖 `update()` 是否跑过

**停机**

```cpp
chassis.normalstop();    // 目标清零 + 滑行停机，保持使能
chassis.emergencyStop(); // 目标清零 + 立刻失能 + 同步施加绕组短接急刹
```

- `normalstop()` 只置状态，实际滑行由周期性调用的 `stop(dt)` 推进
- `emergencyStop()` 的急刹**立即生效**，不依赖 `stop(dt)` 是否被泵
- 两者之后的收尾（判停 / 超时）都由 `stop(dt)` 完成；若此时驱动处于失能状态，
  收尾会**保持**刹住而不恢复 PWM，等下一次 `setEnabled(true)` 再恢复

判停用阈值而非 `== 0.0f`，并有 `STOP_MIN_DWELL_MS` 最小驻留（防止测速窗口刚重置、
还在零阶保持的 0 时被误判成"已停"）与 `STOP_TIMEOUT_MS` 超时兜底，状态机不会卡死。

> 注意 `setEnabled(false)` 会落到**滑行态**并松开已经施加的急刹。
> 需要"停住并保持刹住"请用 `emergencyStop()`。

---

## 参数标定清单

以下均为**占位值**，装机后必须按实车标定：

| 参数 | 位置 | 说明 |
|---|---|---|
| `WHEEL_DIRECTION[4]` | `MecanumParam` | 每轮编码器方向，全 `+1`。悬空给正 PWM，读数为负就改成 `-1` |
| `COURSE_KP/KI/KD` | `ChassisParam` | 航迹环增益（量纲 1/s）。`kd` 建议保持 0 |
| `SPIN_KP/KI/KD` | `ChassisParam` | 自旋环增益 |
| `MAX_TRANSLATE_SPEED` | `ChassisParam` | 平移速度上限 m/s |
| `MAX_SPIN_RATE` / `MAX_OMEGA_CORRECTION` | `ChassisParam` | 自旋速度 / 航迹修正量上限 rad/s |
| `STOP_LINEAR_EPS` / `STOP_ANGULAR_EPS` | `ChassisParam` | 判停阈值 |
| `STOP_MIN_DWELL_MS` / `STOP_TIMEOUT_MS` | `ChassisParam` | 刹车最小驻留 / 超时兜底 |
| `SPEED_WINDOW_S` | `MecanumParam` | 测速窗口，默认 5 ms |
| `MAX_WHEEL_SPEED` | `MotorParam`（[`core/interfaces/IWheelDriver.hpp`](core/interfaces/IWheelDriver.hpp)） | 单轮转速上限 rps，需与 `PWM_MAX` 协同验证 |
| `PWM_MAX` | `MotorParam`（[`core/interfaces/IWheelDriver.hpp`](core/interfaces/IWheelDriver.hpp)） | **必须等于 TIM1 的 `ARR`(8399)**，否则占空比满量程对不上 |
| 引脚映射 | [`boards/chassis_test/BoardPins.h`](boards/chassis_test/BoardPins.h) | 电机 IN1/IN2 与编码器定时器；换接线只改这里 |

---

## 后续计划

| 方向 | 目标 | 落点 |
|---|---|---|
| **蓝牙遥控** | 接入蓝牙串口模块（透传 / BLE），把遥控指令作为输入源喂给 `ChassisController`，替代目前写死在代码里的目标值 | `drivers/`（链路驱动）+ `apps/`（指令解析与映射） |
| **数据回传** | 周期性打包轮速、位姿、PID 中间量上传上位机，用于调参与曲线分析；需要先定一套帧格式 | `apps/` + 计划中的 `modules/` |
| **`modules/` 下沉** | 把现在散在 `apps/` 里可复用的部分（telemetry 组包、遥控帧解析、参数表）下沉为与具体业务无关的 `modules/` 层，依赖方向仍然只能自上而下 | 新增 `modules/` |

> `modules/` 目前**还不存在**，是计划中的一层，不是现存目录。

---

## 已知限制

- **`main.c` 尚未接入**：控制器还没放进初始化流程与控制中断，`chassis_test.elf`
  目前是一个"能编译、能烧录、但不驱动电机"的空壳。
- **没有 IMU 驱动**：`IHeadingSensor` 只有接口，仓库里没有实现类。
  要跑闭环得先按手头的 IMU 型号补一个 `drivers/` 下的实现。
- **`apps/` 仍依赖 HAL**：`ChassisController.cpp` 直接用了 `HAL_Delay` / `HAL_GetTick` /
  `__get_PRIMASK`，所以控制层还做不到纯 PC 编译。这几个调用应当在 `modules/` 下沉时
  换成可注入的时钟/临界区抽象。
- **等比限幅会同时压缩两个通道**：平移速度接近上限时，航迹环的角速度修正量也会被一起压小，
  表现为"跑得越快越转不动"。这是保证四轮比例不偏的固有代价，调参时要留足余量。
- **航迹环抗饱和看不到下游限幅**：`limitWheelSpeed()` 在角速度环之后，修正量被压小时
  角速度环感知不到，积分可能继续累积；目前靠 `COURSE_INTEGRAL_LIMIT` 兜上界，不要设太大。
- **测速窗口降低反馈带宽**：默认 5 ms 窗口把量化噪声从约 15% 降到约 3%，
  代价是反馈带宽降到约 200 Hz。速度环的 `kd` 需按新带宽重调。
- **`starm-clang.cmake` 未验证**：默认构建全部走 GCC（已实测），clang 通路尚未跑通验证。

---

## 参与与约定

- **单位与命名**：见[坐标系与单位约定](#坐标系与单位约定)。`_rps` 是转/秒，`_radps` 是 rad/s，不要混。
- **分层**：`core/` 不许出现 STM32 类型；`drivers/` 绑器件不绑板子；
  `boards/` 是唯一允许出现**引脚定义与外设句柄**的地方。改动前先确认依赖方向没被反向包含。

  > 现状例外：`apps/app_mecanum_test/ChassisController.cpp` 直接 `#include "stm32f4xx_hal.h"`
  > 取 `HAL_Delay` / `HAL_GetTick` / `__get_PRIMASK`。这让 `apps/` 与 HAL 粘上了，
  > 想把它搬到 PC 上跑仿真时得先把这几个调用抽出去（见「后续计划」里的 `modules/` 下沉）。
- **板级契约**：`drivers/` 与 `apps/` 只许 include `BoardPins.h`，
  不许直接 include `core/Inc/tim.h` / `main.h` / `gpio.h`。守住这条，换组合才只需要换一份头文件。
- **`.ioc` 视作二进制**：不要在同一组合的两个分支上同时改，合并冲突无法手工解决。

---

## 声明

### AI 辅助

本项目的设计、实现与调试由作者完成。**AI（DeepSeek）参与的部分是按本工程的工程规范
进行代码审查**——分层与依赖方向、命名与单位约定、接口契约、注释与文档完整性等，
并协助整理本文档。

AI 不作为原始设计决策的最终拍板方，也不产出未经作者复核就直接合入的代码。

### 仅用于学习交流

本仓库为**个人学习与技术交流**用途，不是产品级实现，不保证安全性、可靠性与实时性。
套用到实车（尤其是竞赛、载人或任何涉及人身安全的场景）之前，请自行完成完整的
电气检查、失效分析与实机验证，风险自负。

### 许可

本仓库自身以 [MIT 许可证](LICENSE) 发布，Copyright (c) 2026 泠熙语云 (lingxi)。
可自由学习、修改、二次分发，保留版权与许可声明即可。

`third_party/stm32/` 下的 HAL 与 CMSIS 为 ST 官方授权代码，**不适用** MIT 条款，
各自遵循同目录的 `LICENSE.txt`（HAL：BSD-3-Clause；CMSIS：Apache-2.0），
版权归 STMicroelectronics / ARM 所有。
