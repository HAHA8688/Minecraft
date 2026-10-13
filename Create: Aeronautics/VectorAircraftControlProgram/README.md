# VectorAircraftControlProgram

## 项目简介

本项目为运行于 **ComputerCraft: Tweaked** 的 Lua 飞控程序，面向 **Create: Aeronautics** 载具，提供增稳、姿态控制、速度控制与自动驾驶功能。飞控主循环频率约为 20 Hz，通过姿态传感器、高度传感器及六路速度传感器获取飞行状态，支持串级 PID 控制器与前馈补偿，使用伪逆矩阵分配六维控制指令。

项目包含独立的仪表显示程序。该程序通本地过 rednet 接收飞控数据（替代不支持的IPC），并在外部显示器上绘制地平仪及高度、气压、三轴速度等飞行信息。

## 功能特性

- **姿态解算**：以角速率积分四元数作为基础姿态，在姿态接近水平时与万向节传感器数据融合，以抑制万向节锁与漂移；万向节安装方向可配置。
- **多级 PID 控制**：
  - 姿态环：偏航、俯仰、横滚姿态控制；
  - 角速度阻尼环：偏航、俯仰、横滚角速率控制；
  - 世界系速度环：gx、gy、gz 速度控制；
  - 高度环：含自动重力前馈；
  - 支持位置式与增量式控制，具备积分分离、微分低通滤波、积分限幅与输出限幅。
- **前馈补偿**：
  - 重力前馈，支持自适应学习；
  - 静态姿态前馈，用于悬停状态下俯仰与横滚配平自适应；
  - Z 轴机动前馈、X 轴机动前馈与 Y 轴机动前馈。
- **控制分配**：采用 6×N 伪逆矩阵，将 `[τ_roll, τ_pitch, τ_yaw, fx, fy, fz]` 映射至各执行器；支持输出归一化、整数化，并可在运行时切换输出范围（空挡、低挡、高挡）。
- **传感器融合**：读取高度、气压、万向节角度，以及六路速度传感器数据。X、Y、Z 三轴各配置两路速度传感器，用于差分测量角速率与线速度。
- **按键与继电器输入**：支持 `linked_typewriter` 键盘输入及 `redstone_relay` 模拟/数字输入，用于挡位与离合器控制。
- **进程守护**：`startup.lua` 通过 rednet 心跳监控飞控与仪表进程，异常退出时自动重启。
- **仪表显示**：`instrument.lua` 使用 `monitor.blit` 绘制地平仪，天地区域颜色分界随俯仰与横滚变化，并在左侧区域显示飞行数据。
- **传感器数据记录**：`test.lua` 以 10 Hz 频率记录姿态与角速率至日志文件，便于调试。

## 文件结构

```text
VectorAircraftControlProgram/
├── startup.lua                       # 进程守护：启动并监控飞控与仪表程序
├── README.md
└── ProgramFiles/
    ├── config_op.lua                 # 配置加载、反序列化、默认值合并、外设查找
    ├── FlightController/
    │   ├── config.cfg                # 飞控配置
    │   ├── flight_controller.lua     # 飞控主程序
    │   ├── control_allocator.lua     # 伪逆矩阵控制分配器
    │   ├── pid_controller.lua        # PID 控制器（位置式/增量式）
    │   ├── sensor_reader.lua         # 传感器读取与姿态解算
    │   ├── math_utils.lua            # 四元数与数学工具
    │   └── test.lua                  # 传感器数据记录测试脚本
    └── Instrument/
        ├── config.cfg                # 仪表配置
        └── instrument.lua            # 仪表主程序
```

## 环境依赖

| 依赖项 | 说明 | 必需性 |
|--------|------|--------|
| Minecraft Java Edition | 1.21.1 | 必需 |
| NeoForge | 适用于 Minecraft 1.21.1 的 NeoForge 加载器 | 必需 |
| Create | 对应 1.21.1 / NeoForge 版本 | 必需 |
| Sable | 对应版本 | 必需，Create: Aeronautics 的前置模组 |
| Create: Aeronautics | 对应版本 | 必需 |
| ComputerCraft: Tweaked | 对应版本 | 必需，提供 Lua 运行环境与外设 API |
| Copycats | 对应版本 | 可选 |

所有模组版本均应与 Minecraft 1.21.1 及 NeoForge 匹配。Create: Aeronautics 需要 Create 与 Sable 作为前置模组。Copycats 为可选模组，不参与飞控核心逻辑。

## 硬件与外设约定

### 坐标系

采用 FRD 机体坐标系：

- +X：机头方向；
- +Y：机体右侧；
- +Z：机腹方向，即重力方向。

世界系中的角速率、速度与前馈向量均遵循该约定。

### 所需外设

| 外设类型 | 数量 | 用途 |
|----------|------|------|
| `linked_typewriter` | 1 | 键盘输入 |
| `redstone_relay` | 1 | 挡位模拟输入与离合器数字输入 |
| `gimbal_sensor` | 1 | 俯仰与横滚姿态 |
| `altitude_sensor` | 1 | 高度与气压 |
| `velocity_sensor` | 6 | 三轴速度差分与角速率 |
| `monitor` | 1 | 仪表显示 |
| 执行器 | N | 由 `ActuatorList` 配置，示例为 `Create_RotationSpeedController_*` |
| `modem` | 1 | rednet 通信，用于进程守护与仪表数据传输 |

具体外设名称与类型应以游戏内实际注册名为准，并可在配置文件中修改。

## 部署与运行

1. 将 `VectorAircraftControlProgram` 文件夹置于 ComputerCraft: Tweaked 电脑的根目录。
2. 确认电脑已连接 `modem`，并设置正确的 `modem_side`。
   - 须注意：`startup.lua` 中 `rednet.open("bottom")` 为硬编码。若调制解调器未安装在电脑底面，应同步修改该语句。
3. 根据实际外设名称与安装位置，编辑以下文件：
   - `ProgramFiles/FlightController/config.cfg`
   - `ProgramFiles/Instrument/config.cfg`
4. 运行 `startup.lua`。该脚本将分别在后台启动飞控与仪表程序，并周期性接收心跳；当进程异常退出时，将自动重启。

若仅需调试传感器，可在 `ProgramFiles/FlightController/` 目录下运行 `test.lua`。该脚本将以 10 Hz 频率把姿态与角速率记录至 `sensor_log.txt`。

## 配置说明

### 飞控配置（`ProgramFiles/FlightController/config.cfg`）

| 配置项 | 含义 |
|--------|------|
| `sleep_time` | 主循环周期，单位为秒，默认值为 0.05 |
| `height_deadband` / `angle_err_deadband` / `angle_rate_err_deadband` / `speed_err_deadband` | 各控制通道死区 |
| `pre_grav_feed` / `auto_grav_feed_sens` / `auto_grav_feed_deadband` | 重力前馈初值与自适应参数 |
| `yaw_rate_damping_kp` / `pit_rate_damping_kp` / `rol_rate_damping_kp` | 刹车阻尼增益 |
| `thro_sens` / `atti_sens` | 油门与姿态按键灵敏度 |
| `pit_static_ff_sens` / `rol_static_ff_sens` / `*_ff_max` | 静态姿态前馈自适应系数与限幅 |
| `pit_z_acc_ff_sens` / `rol_z_acc_ff_sens` / `*_ff_max` | Z 轴机动前馈参数 |
| `pit_x_acc_ff_sens` / `rol_y_acc_ff_sens` / `*_ff_max` | X 轴与 Y 轴机动前馈参数 |
| `x_sp_ctrl_ff_max` / `y_sp_ctrl_ff_max` / `z_sp_ctrl_ff_max` | 速度控制前馈上限 |
| `sim_resistance` | 速度指令的虚拟阻力，用于滑行衰减 |
| `modem_side` | 调制解调器所在面 |
| `sensor_data_protocol` / `heartbeat_protocol` / `exit_protocol` | rednet 协议名 |
| `LinkedTypewriter.key_codes` | 键位码表；若留空，则使用程序内置默认值 |
| `RedstoneRelay` | 红石继电器名称，以及 `throttle_side` 与 `clutch_side` |
| `Actuators` | 执行器列表、默认方法、伪逆矩阵、输出上下限与是否整数化 |
| `Sensors` | 各类传感器名称与方向、传感器间距、低通滤波系数 |
| `PIDControllers` | 各 PID 控制器的 kp、ki、kd、限幅、积分分离与微分滤波系数等 |

### 仪表配置（`ProgramFiles/Instrument/config.cfg`）

| 配置项 | 含义 |
|--------|------|
| `MainMonitor.name` | 显示器外设名称；留空则自动查找 |
| `modem_side` | 调制解调器所在面 |
| `sensor_data_protocol` / `heartbeat_protocol` / `exit_protocol` | rednet 协议名 |
| `inf_area_bc` / `inf_area_tc` | 左侧信息区背景色与文字色 |
| `sky_bc` / `ground_bc` | 地平仪天空与地面颜色 |
| `text_offset_x` / `text_offset_y` | 文本偏移量 |

## 控制原理概述

### 1. 传感器与姿态解算（`sensor_reader.lua`）

六路速度传感器经差分计算得到 `yaw_rate`、`pit_rate` 与 `rol_rate`，线速度经低通滤波处理。角速率用于积分四元数，从而获得连续姿态。当俯仰与横滚合成角度较小（小于约 1.4 rad）时，使用万向节传感器数据构造水平姿态四元数，并根据接近程度动态确定融合权重。

姿态融合仅作用于 `w`、`x`、`y` 分量，不融合 `z` 分量。该设计用于避免陀螺仪积分偏航噪声自激励所导致的偏航漂移。

### 2. 飞控主循环（`flight_controller.lua`）

飞控主循环频率约为 20 Hz，每轮执行以下步骤：

1. 刷新传感器，获取姿态四元数、角速率、机体/世界速度、加速度与重力向量。
2. 计算姿态环 PID。以误差四元数 `err_quat = conj(cur) * tar` 的分量驱动姿态环，输出目标角速率。
3. 处理离合器与挡位。离合器压下时跳过控制；切换挡位时重置速度环 PID，并调整分配器输出范围。
4. 解析按键。`linked_typewriter` 输入经 `keyCodesProcess` 转换为按下计数，并处理巡航、刹车、回中、重启等状态。
5. 生成速度指令。按键增量累加至世界系目标速度；未启用巡航时，按 `sim_resistance` 进行指数衰减；随后旋转至机体系参与速度环。
6. 执行前馈补偿。重力前馈由高度环输出与自适应学习得到；静态姿态前馈在角速率较低且加速度较小时自适应学习；Z、X、Y 机动前馈在存在对应方向加速度且姿态稳定时学习。
7. 执行角速度阻尼。刹车标志激活时，在输出上叠加角速率阻尼；转速过零后自动复位并锁定目标姿态。
8. 执行控制分配。将 `[τ_roll, τ_pitch, τ_yaw, fx, fy, fz]` 传递给 `control_allocator`。

### 3. 控制分配（`control_allocator.lua`）

对每个执行器，按伪逆矩阵对应行进行六维点积，得到原始输出。随后依据最大输出与 `max_output` / `min_output` 进行统一缩放，以保证输出不越界且不超出范围。若启用 `int_output`，结果将四舍五入为整数。运行时可通过 `setOutputRange` 切换空挡、低挡与高挡输出范围。

### 4. PID 控制器（`pid_controller.lua`）

位置式 PID 形式为 `kp·e + ki·∫e + kd·de/dt`，其中微分项采用一阶低通滤波，积分项具备限幅与积分分离功能。增量式 PID 输出差分形式，不维护积分项。该模块提供 `reset`、`setPIDargs`、`setDt`、`setAlpha`、`setOutputRange` 与 `setIntegralRange` 等运行时接口。

### 5. 仪表（`instrument.lua`）

仪表程序通过 `rednet.receive(data_protocol)` 接收飞控发送的七元数据包，并使用 `monitor.blit` 一次性写入文本、前景色与背景色。地平仪绘制分为两种情况：

- 小横滚：直接依据俯仰角计算天地分界行；
- 大横滚：依据 `tan(roll)` 计算每行分界列，形成旋转地平线。

左侧固定区域显示气压百分比、高度、VX/VY/VZ 速度以及俯仰与横滚角度。程序使用 `output_buffer` 缓存上一帧内容，仅在内容变化时重绘。

### 6. 进程守护（`startup.lua`）

启动脚本使用 `shell.run("bg <script>")` 分别在后台启动飞控与仪表程序。主循环以 0.1 s 周期通过 `parallel.waitForAll` 同时接收两路心跳。若某进程超过 `heartbeat_overtime`（3 s）未发送心跳，则发送退出协议并重启该进程。心跳时间戳发生变化时，也会通知旧进程退出，以防止重复运行。

## 默认键位映射

| 功能 | 默认键码 |
|------|----------|
| 前进 / 后退 | 87 / 83 |
| 左移 / 右移 | 65 / 68 |
| 上升 / 下降 | 32 / 340 |
| 偏航左 / 右 | 81 / 69 |
| 抬头 / 低头 | 265 / 264 |
| 左滚 / 右滚 | 263 / 262 |
| 姿态回中 | 258 |
| 刹车 | 82 |
| 巡航 | 342 |
| 重启 | 259 |

键码为 ComputerCraft: Tweaked `keys` API 常量。多个按键同时按下时，输入将叠加。例如同时按下抬头与低头等价于角速度刹车。若 `config.cfg` 中 `LinkedTypewriter` 为空表，则使用 `flight_controller.lua` 内置的默认键位。

## 调试与数据记录

`ProgramFiles/FlightController/test.lua` 为精简的传感器记录脚本。该脚本加载配置并初始化 `sensor_reader`，随后以 10 Hz 频率循环刷新传感器，并将时间戳、`yaw_rate`、`pit_rate`、`rol_rate` 与四元数写入 `sensor_log.txt`。该脚本适用于调整 PID 参数或排查传感器问题。

## 许可证

本项目未指定许可证。如需使用或分发，请先联系作者。
