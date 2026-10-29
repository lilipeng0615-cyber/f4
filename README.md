# RoboMaster 电控基础框架（STM32F407 / 大疆 C 板）

基于 **STM32F407IGH6** 的机器人电控框架，采用 **bsp / module / application** 三层结构，支持多兵种机器人快速部署与移植。

构建系统为 **CMake + Ninja**，工具链 **arm-none-eabi-gcc**，调试走 **OpenOCD（DAPLink/ST-Link）** 或 J-Link。

> 本工程源自 [HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)，在此基础上完成步进电机接入、VQF 姿态解算、CAN 发送队列重构等改动，详见 [本工程的改动](#本工程的改动)。

***==别忘了仓库根目录下的 `.Doc` 文件夹中有关于配置环境和开发更详尽的说明！==***

---

## 目录结构

```
basic_framework/
├── application/              应用层
│   ├── chassis/              底盘：麦克纳姆 / 全向 / 平衡（条件编译）
│   ├── cmd/                  指令解析与发布（robot_cmd）
│   ├── gimbal/               云台（yaw / pitch）
│   ├── shoot/                发射机构
│   ├── robot.c/.h            整车装配
│   ├── robot_def.h           全局配置：板型、机械参数、视觉链路
│   └── robot_task.h          五个 RTOS 任务
├── bsp/                      板级支持包，11 类外设
├── modules/                  模块层
│   ├── motor/                DJI / DM / HT / LK / 舵机 / 步进
│   ├── algorithm/            PID、卡尔曼、QuaternionEKF、VQF、CRC、user_lib
│   └── ...                   通信、传感器、UI、守护线程
├── Drivers/                  CMSIS + STM32F4xx HAL
├── Middlewares/              FreeRTOS + CMSIS-DSP
├── .Doc/                     开发文档（8 篇）
├── CMakeLists.txt            构建脚本（显式源文件列表）
├── CMakePresets.json         Debug / Release 预设
└── basic_framework.ioc       CubeMX 工程
```

## 功能清单

### bsp（板级支持包）

| 类别 | 模块 |
|---|---|
| 通信 | usart、spi、iic、can、usb |
| 数据 | log、flash |
| 功能 | gpio(exti)、pwm、adc |
| 辅助 | dwt（微秒级计时） |

### module（模块层）

| 类别 | 模块 |
|---|---|
| 电机 | **DJI**、**DM 达妙**、**HT 海泰**、**LK 瓴控**、**servo 舵机**、**step_motor 步进（张大头 ZDT，CAN）** |
| 通信 | can_comm（多板）、unicomm、standard_cmd、master_machine、referee（裁判系统 v2.0.0）、bluetooth、remote（DT7/DR16） |
| 传感器 | BMI088、ist8310、encoder、TFminiPlus |
| 功能 | super_cap（超级电容）、oled、alarm（蜂鸣器）、daemon（守护线程）、message_center（发布订阅） |
| 算法 | controller(PID)、kalman_filter、QuaternionEKF、**VQF_filter**、crc8/crc16、user_lib |

### application（应用层）

| 应用 | 说明 |
|---|---|
| `robot_cmd` | 从遥控器 / 上位机 / 传感器取控制信号，解析成执行单元目标并发布 |
| `chassis` | 麦克纳姆 / 全向 / 平衡底盘，含逆运动学与功率控制 |
| `gimbal` | 云台 yaw / pitch 双轴串级控制 |
| `shoot` | 拨盘 + 摩擦轮 |

## 硬件配置

当前 `application/robot_def.h` 的配置为**单板标准步兵**：

| 宏 | 当前值 | 说明 |
|---|---|---|
| `ONE_BOARD` | 启用 | 单板控制整车（另有 `CHASSIS_BOARD` / `GIMBAL_BOARD` 双板方案） |
| `VISION_USE_VCP` | 启用 | 视觉数据走 USB 虚拟串口（另有 `VISION_USE_UART`） |
| `YAW_CHASSIS_ALIGN_ECD` | 2711 | 云台与底盘同向时的 yaw 编码器值 |
| `PITCH_HORIZON_ECD` | 3412 | 云台水平时 pitch 编码器值 |
| `WHEEL_BASE` / `TRACK_WIDTH` | 350 / 300 mm | 纵向轴距 / 横向轮距 |
| `RADIUS_WHEEL` | 60 mm | 轮半径 |
| `REDUCTION_RATIO_WHEEL` | 19.0 | M3508 减速比 |
| `REDUCTION_RATIO_LOADER` | 36.0 | M2006 拨盘减速比 |
| `NUM_PER_CIRCLE` | 10 | 拨盘一圈装载量 |

改动机械结构后，**标定值（`*_ECD`）和几何参数（轮距、轴距、半径、减速比）必须同步修改**，否则姿态解算和底盘运动学都会错。

当前 CubeMX 已启用的外设：

```
CAN1 / CAN2        SPI1 / SPI2        USART1 / USART3 / USART6
I2C2 / I2C3        ADC1               DAC
TIM1 / TIM4 / TIM5 / TIM8 / TIM10
RNG / RTC / CRC    DMA                USB (CDC)
```

## 构建

### 依赖

| 工具 | 版本 | 备注 |
|---|---|---|
| CMake | ≥ 3.22 | |
| Ninja | 任意近期版本 | `CMAKE_MAKE_PROGRAM` 需指向它 |
| arm-none-eabi-gcc | 建议 10.3 以上 | ARM GNU Toolchain 官方安装包即可 |

装好之后 `cmake --preset Debug` 一键配置，`cmake --build --preset Debug` 编译。

本工程**不使用** mingw32-make / Msys2。根目录的 `Makefile` 与 `Makefile.upgrade` 是上游遗留的备选方案，已不再维护，机器上没有 make 直接忽略即可。

### VSCode 方式

装 **CMake Tools** 插件 → 底部状态栏选 `Debug` preset → 点 Build（或 `Ctrl+Shift+B`）。

### 命令行方式

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

链接阶段由 `-Wl,--print-memory-usage` 打印各内存区占用，`POST_BUILD` 再生成 `.hex` / `.bin` 并报告大小。关注 FLASH / RAM 的百分比即可。

### ⚠️ 新增源文件必须手动登记

`CMakeLists.txt` 的 `target_sources` 是**显式列表，不是 GLOB**：

- 新建 `.cpp` / `.c` → 在 `target_sources(...)` 里加一行
- 新建目录 → 在 `target_include_directories(...)` 里加一行

漏加的典型症状：头文件能找到、声明没问题，但链接期报 `undefined reference to XXX`。改完 `CMakeLists.txt` 后 `cmake --build` 会自动重新 configure，不用手动 clean。

## 烧录与调试

把调试器接到开发板，按 **F5** 开始调试：`launch.json` 里的 `DAPlink (cppdbg)` 会自动启动 OpenOCD、烧录固件并停在 `main`。

只想烧录不调试：

```powershell
openocd -f openocd_dap.cfg -c "program build/Debug/basic_framework.elf verify reset exit"
```

OpenOCD 与 `arm-none-eabi-gdb` 不在 PATH 里时，在 `launch.json` 里显式写全路径（`serverpath` / `gdbPath`）。

J-Link 的配置保留在 `openocd_jlink.cfg` 与 `stm32.jflash`。

## 任务结构

`application/robot_task.h` 创建五个 RTOS 任务：

| 任务 | 入口函数 | 职责 |
|---|---|---|
| INS | `StartINSTASK` | 惯性导航与姿态解算（BMI088 → VQF / QuaternionEKF） |
| MOTOR | `StartMOTORTASK` | 电机控制与 CAN 收发 |
| DAEMON | `StartDAEMONTASK` | 守护线程，离线检测与报警 |
| ROBOT | `StartROBOTTASK` | 应用层主循环（cmd / chassis / gimbal / shoot） |
| UI | `StartUITASK` | 裁判系统 UI 绘制 |

`bsp` 提供了把回调从中断转移到任务的封装，用于保证实时性；`modules` 和 `application` 各自按需创建定时任务或事件驱动任务。

## 设计约定

1. **三层单向依赖**：`application → module → bsp`，反向不成立。底层需要回调上层时，通过初始化时注册的**函数指针**完成，不在 bsp 里 include 上层头文件。
2. **实例化风格**：每个 bsp 都有一个 `XXXInstance`，通过 `XXXRegister(&config)` 拿到实例指针（C 里模拟 `this`），后续调用都传入该指针。
3. **app 之间用 pub-sub**：`message_center` 解耦，避免应用互相 include。
4. **配置集中在 `robot_def.h`**：板型、机械参数、视觉链路都在这里切换。

## 本工程的改动

相对上游 `basic_framework`：

- **步进电机接入**：新增 `modules/motor/step_motor`，支持张大头（ZDT）系列，走 CAN 通信
- **姿态解算**：引入 VQF（`modules/algorithm/VQF_filter.c`），与原有 `QuaternionEKF` 并存，可按需切换
- **CAN 发送重构**：发送改为循环队列，不再忙等；开启 CAN1_TX / CAN2_TX / SCE 中断；`CANTransmit` 参数列表重新封装并做上层适配
- **裁判系统**：协议更新至 v2.0.0，封装图传链路
- **达妙电机驱动**：更新 DM 电机驱动
- **构建系统**：由 Makefile 迁移到 CMake + Ninja + CMakePresets
- **文档**：README 重写，清理上游示例中不适用于本车的内容

## 文档

`.Doc` 目录下有 **8** 篇 markdown：

- [Bug_Report](.Doc/Bug_Report.md) — 提交 issue 的模板范例
- [TODO](.Doc/TODO.md) — 框架后续开发计划
- **[VSCode+Ozone使用方法](.Doc/VSCode+Ozone使用方法.md)** — **上手必看**，环境配置与工具链原理
- [合理地进行PID参数整定](.Doc/合理地进行PID参数整定.md) — 经验准则、前馈控制、扰动消除
- [如何定位bug](.Doc/如何定位bug.md) — 调试器技巧与错误复现
- [必须做&禁止做](.Doc/必须做&禁止做.md) — 字如其名
- **[架构介绍与开发指南](.Doc/架构介绍与开发指南.md)** — **开发必看**，编码与命名规范、文件树
- [让VSCode成为更称手的IDE](.Doc/让VSCode成为更称手的IDE.md) — 插件与个性化配置

每个 `bsp/` 和 `modules/` 的子目录里还有各自的说明文档，包含接口说明和测试用例。

## 运行单个 bsp/module 测试

硬件查错、连线检查、新人教学时可以脱离整车单独测某个模块：

1. 在 `main.c` 里注释掉 `RobotInit()`，改为调用对应模块的初始化
2. 包含该模块头文件，按它的说明文档初始化
3. 可注释掉 RTOS 初始化，直接在 `while(1)` 里测试；也可用 bsp 提供的定时任务
4. 编译、下载、运行

## 后续计划

- `.Doc/TODO.md` 里的功能增强项
- 把散落各处的配置统一收拢到 `application/robot_def.h`
- 为三个层级补入门级教程，支持各模块独立运行
- 优化 pub-sub 性能，把 app 任务改成**状态机 + 事件驱动**的回调机制
- 为框架编写 ROS driver，通过 USB 连接 NUC，合并视觉与电控工作流

## 致谢

框架设计与实现源自 **[HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)**（湖南大学岳麓电控）。

其上游参考了哈尔滨工业大学（深圳）南工骁鹰战队的 EC_framework 与 RoboMaster 官方 RoboRTS-firmware；姿态解算改进自哈尔滨工程大学创梦之翼的四元数 EKF；裁判系统数据解析移植自深圳大学 RoboPilot 2021 年电控英雄开源代码。
