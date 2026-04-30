# 角度 PID 控制器（CC:Tweaked）

基于 **PID 控制算法** 的旋转角度闭环控制脚本，适用于 **CC:Tweaked** 环境下机械动力航空学的旋转轴承控制。  
通过键盘实时调节目标角度，利用 PID 控制器驱动 **旋转轴承（Swivel Bearing）** 精确（也许）到达并保持指定角度。

---

## 文件说明

| 文件 | 说明 |
|------|------|
| `PIDController.lua` | 通用 PID 控制器模块。|
| `angleController.lua` | 实现角度闭环控制，并响应键盘输入调整目标。 |

---

## 视频链接：
【[机械动力：航空学] 基于CC：T的模拟轴承控制器】 

https://www.bilibili.com/video/BV1ds9YBXEbh/?share_source=copy_web&vd_source=44911cf6325356233e85db26f07d66e6

---

## 环境要求

- **Minecraft** 1.21.1 NeoForge
- **CC:Tweaked**
- **Create** 及 **Create: Aeronautics**
- 需摆放以下外设：
  - `Create_RotationSpeedController` —— 转速控制器
  - `swivel_bearing` —— 旋转轴承
  - `linked_typewriter` —— 红石打字机
  - `monitor` —— 监视器

> 如果启动时检测到存在外设缺失，程序会退出。

---

## 安装与运行

1. 将 `PIDController.lua` 和 `angleController.lua` 放入同一目录下。
2. 在 CC:Tweaked 终端中运行输入angleController运行
3. 屏幕上会显示启动信息，随后进入控制循环。

---

## 操作说明

| 按键 | 功能 |
|------|------|
| `↑` 上方向键 | 增加目标角度 |
| `↓` 下方向键 | 减少目标角度 |
| `Shift + ↑` | 粗调目标角度（步长 0.1 rad） |
| `Shift + ↓` | 粗调减少目标角度 |
| `Ctrl + W` | 安全退出程序 |

显示屏上实时显示：
- `C:` 当前实际角度（度）
- `T:` 目标角度（度）
- `S:` PID 输出的目标转速

角度值均以弧度内部计算，显示时转换为度。

---

## PID 控制器参数

`PIDController.lua` 提供了完整的 PID 控制器实现，可通过初始化参数配置，也可运行时调用方法修改。

**默认参数：**

```lua
kp = 1.0, ki = 0.5, kd = 0.3,
dt = 0.05,          -- 控制周期（秒）
max_i = 1, min_i = -1,      -- 积分限幅
max_output = 1, min_output = -1,  -- 输出限幅（转速范围）
alpha_d = 0.8       -- 微分项低通滤波系数（0~1，越小滤波越强）
