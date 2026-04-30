local PIDController = require("PIDController")

-- 查找连接的外设
local RotationSpeedController = peripheral.find("Create_RotationSpeedController")  -- 控制转速
local SwivelBearing = peripheral.find("swivel_bearing")                           -- 获取当前角度
local LinkedTypewriter = peripheral.find("linked_typewriter")                     -- 获取键盘输入
local Monitor = peripheral.find("monitor")                                        -- 显示器

-- 存在外设缺失，直接退出
if RotationSpeedController == nil or SwivelBearing == nil or LinkedTypewriter == nil or Monitor == nil then
    return end

-- 启动提示
print("Start successfully!")

Monitor.clear()
Monitor.setCursorPos(1,1)
Monitor.write("Start successfully!")
sleep(1.0)
Monitor.clear()

-- 控制周期（秒）
local dt = 0.05
local pressed_key_codes = nil        -- 存储按下的键码列表
local target_speed = 0               -- PID输出的目标转速
local target_angle = 0               -- 期望的角度（弧度）
local current_angle = 0              -- 当前实际角度（弧度）
local error = 0                      -- 角度误差
local PI = 3.141592654

-- PID控制器参数配置
local pid_args = {
    kp = 40.0, ki = 22.0, kd = 2.0,   -- 比例、积分、微分系数
    dt = dt,                          -- 采样时间
    max_i = 1, min_i = -1,            -- 积分限幅
    max_output = 256, min_output = -256, -- 输出限幅（转速范围）
    alpha_d = 0.8,                    -- 微分项低通滤波系数
}

-- 创建角度PID控制器实例
local angleController = PIDController.new(pid_args)

-- 在表中查找是否存在目标值
local function search_value(table, target_value)
    for _, value in pairs(table) do
        if value == target_value then return true end
    end
    return false
end

-- 将数值限制在 [min_value, max_value] 区间内
local function clump(value, min_value, max_value)
    if min_value >= max_value then return value
    elseif value > max_value then return max_value
    elseif value < min_value then return min_value
    else return value end
end

-- 将弧度限制到 (-PI, PI] 区间（圆形规范化）
local function circle_clump(rad)
    if rad > PI then return rad - 2 * PI
    elseif rad < -PI then return 2 * PI + rad
    else return rad end
end


while true do
    -- 获取当前轴承角度（弧度）
    current_angle = SwivelBearing.getTargetAngleRad()
    -- 计算目标角度与当前角度的误差并处理循环边界
    error = ((target_angle - current_angle + PI) % (2 * PI)) - PI
    if math.abs(error) < 0.0001 then error = 0 end
    print(error)  -- 调试输出误差值

    -- PID更新：输入误差，输出需要的转速
    target_speed = angleController: update(error)

    -- 小转速归零，避免微颤
    if math.abs(target_speed) < 0.005 then target_speed = 0
    -- elseif target_speed < 0.99 and target_speed > 0 then target_speed = 0.99
    -- elseif target_speed > -0.99 and target_speed < 0 then target_speed = -0.99
    end
    -- 将目标转速应用到旋转控制器
    RotationSpeedController.setTargetSpeed(target_speed)

    -- 更新显示器：显示当前角度、目标角度、当前转速（角度单位转换为度）
    Monitor.clear()
    Monitor.setCursorPos(1,1)
    Monitor.write("C:".. current_angle * 180 / PI)
    Monitor.setCursorPos(1,2)
    Monitor.write("T:".. target_angle * 180 / PI)
    Monitor.setCursorPos(1,3)
    Monitor.write("S:".. target_speed)

    -- 获取键盘按下的键码列表
    pressed_key_codes = LinkedTypewriter.getPressedKeyCodes()

    -- 如果同时按下 Ctrl + W，则退出程序
    if pressed_key_codes[1] == 341 and pressed_key_codes[2] == 87 then break end

    -- 若有键被按下，处理角度调节
    if next(pressed_key_codes) ~= nil then
        --[[
        Monitor.clear()
        Monitor.setCursorPos(1,1)
        Monitor.write(textutils.serialize(pressed_key_codes))
        ]]

        -- 上方向键（265）增加目标角度
        if search_value(pressed_key_codes, 265) then
            -- 同时按下左Shift（340）为粗调，否则微调
            if search_value(pressed_key_codes, 340) then
                target_angle = target_angle + 0.1
            else
                target_angle = target_angle + 0.01
            end
        -- 下方向键（264）减小目标角度
        elseif search_value(pressed_key_codes, 264) then
            if search_value(pressed_key_codes, 340) then
                target_angle = target_angle - 0.1
            else
                target_angle = target_angle - 0.01
            end
        end

        -- 将目标角度限制到 (-PI, PI] 范围内
        target_angle = circle_clump(target_angle)
    end

    -- 等待一个控制周期
    sleep(dt)
end

Monitor.clear()
Monitor.setCursorPos(1,1)
Monitor.write("Goodbye!")
sleep(0.5)
Monitor.clear()

print("Goodbye!")