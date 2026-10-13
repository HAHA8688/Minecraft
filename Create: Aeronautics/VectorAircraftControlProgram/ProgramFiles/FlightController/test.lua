-- main.lua
-- 精简版：配置加载 + 传感器初始化 + 以 10Hz 记录传感器数据
package.path = package.path .. ";/ProgramFiles/?.lua"
local configOp = require("config_op")
local sensorReader = require("sensor_reader")

-- 默认配置（合并时使用，与 sensor_reader 的默认设置匹配）
local default_config = {
    sleep_time = 0.1,
    err_deadband = 0.01,
    pre_grav_feed = 0,
    thro_sens = 0.7,
    atti_sens = 0.05,
    sim_resistance = 0.05,
    modem_side = "back",
    sensor_data_protocol = "SensorData",
    heartbeat_protocol = "FlightHeartbeat",
    exit_protocol = "FlightExit",
    LinkedTypewriter = {},
    NavigationTable = {},
    RedstoneRelay = {
        name = "",
        throttle_side = "front"
    },
    Engines = {
        XEngineInYp = { k = 1, dis_x = 0, dis_y = 1, name = "" },
        XEngineInYn = { k = 1, dis_x = 0, dis_y = -1, name = "" },
        YEngineInCe = { k = 1, dis_x = 0, dis_y = 0, name = "" },
        ZEngineInXpYp = { k = 1, dis_x = 1, dis_y = 1, name = "" },
        ZEngineInXpYn = { k = 1, dis_x = 1, dis_y = -1, name = "" },
        ZEngineInXnYp = { k = 1, dis_x = -1, dis_y = 1, name = "" },
        ZEngineInXnYn = { k = 1, dis_x = -1, dis_y = -1, name = "" },
    },
    Sensors = {
        AltitudeSensor = {},
        GimbalSensor = { dir = 0 },
        VelocitySensors = {
            XVeSensorInYp = { dir = 0, name = "" },
            XVeSensorInYn = { dir = 0, name = "" },
            YVeSensorInXp = { dir = 0, name = "" },
            YVeSensorInXn = { dir = 0, name = "" },
            ZVeSensorInYp = { dir = 0, name = "" },
            ZVeSensorInYn = { dir = 0, name = "" }
        },
        x_sensors_dis = 1,    -- 需要正数，否则 new() 会失败
        y_sensors_dis = 1,
        z_sensors_dis = 1,
        lowpass_alpha = 0.95, -- 可选
    },
    PIDControllers = {
        heightPidCtrl = { dt = 1, kp = 4, ki = 0.3, kd = 0.1, max_output = 128, min_output = -128, max_i = 64, min_i = -64, separation_i = 30, alpha_d = 0.5, is_increment = false },
        yawAttiPidCtrl = { dt = 1, kp = 12, ki = 2, kd = 3, max_output = 10, min_output = -10, max_i = 25, min_i = -25, separation_i = 0.5, alpha_d = 0.8, is_increment = false },
        pitAttiPidCtrl = { dt = 1, kp = 12, ki = 2, kd = 3.5, max_output = 10, min_output = -10, max_i = 25, min_i = -25, separation_i = 0.5, alpha_d = 0.8, is_increment = false },
        rolAttiPidCtrl = { dt = 1, kp = 12, ki = 2, kd = 3.5, max_output = 10, min_output = -10, max_i = 25, min_i = -25, separation_i = 0.5, alpha_d = 0.8, is_increment = false },
        yawRatePidCtrl = { dt = 1, kp = 8, ki = 1, kd = 3, max_output = 200, min_output = -200, max_i = 15, min_i = -15, separation_i = 0.3, alpha_d = 0.9, is_increment = false },
        pitRatePidCtrl = { dt = 1, kp = 10, ki = 1, kd = 4, max_output = 200, min_output = -200, max_i = 15, min_i = -15, separation_i = 0.3, alpha_d = 0.9, is_increment = false },
        rolRatePidCtrl = { dt = 1, kp = 10, ki = 1, kd = 4, max_output = 200, min_output = -200, max_i = 15, min_i = -15, separation_i = 0.3, alpha_d = 0.9, is_increment = false },
        xSpeedPidCtrl = { dt = 1, kp = 15, ki = 3, kd = 0.5, max_output = 55, min_output = -55, max_i = 3, min_i = -3, separation_i = 3, alpha_d = 0.7, is_increment = false },
        ySpeedPidCtrl = { dt = 1, kp = 15, ki = 0.8, kd = 0.5, max_output = 55, min_output = -55, max_i = 3, min_i = -3, separation_i = 0.5, alpha_d = 0.6, is_increment = false },
        zSpeedPidCtrl = { dt = 1, kp = 15, ki = 3, kd = 1, max_output = 55, min_output = -55, max_i = 3, min_i = -3, separation_i = 3, alpha_d = 0.7, is_increment = false }
    }
}

-- 加载用户配置并合并
local config = configOp.load(shell.resolve("./config.cfg"))
local settings = configOp.merge(config, default_config)

-- 初始化传感器
local sr = sensorReader.new(settings.Sensors)
if not sr then
    print("no sensors")
    return
end

print("start")

-- 释放配置变量
config = nil
settings = nil

-- 日志文件路径
local log_file = "sensor_log.txt"
-- 确保文件可写（打开追加模式并关闭）
local f = io.open(log_file, "a")
if f then f:close() end



-- 无限循环：10Hz 记录数据
while true do
    sr:update()  -- 刷新传感器数据
    
    local timestamp = os.epoch("utc")  -- 毫秒级时间戳
    
    -- 正确字段名：yaw_rate, pit_rate, rol_rate
    local yaw_rate = sr.yaw_rate or 0
    local pit_rate = sr.pit_rate or 0
    local rol_rate = sr.rol_rate or 0
    
    -- 四元数 (索引1=w, 2=x, 3=y, 4=z)
    --local q = sr.quaternion or {1, 0, 0, 0}
    local q = sr.quaternion
    local q_str = string.format("%.6f,%.6f,%.6f,%.6f", q[1], q[2], q[3], q[4])
    
    -- 组装行：时间戳: yaw_rate, pit_rate, rol_rate, quaternion;
    local line = string.format("[%d] YR: %.6f, PR: %.6f, RR: %.6f, QN: (%s);", timestamp, yaw_rate, pit_rate, rol_rate, q_str)
    
    -- 写入文件
    local file = io.open(log_file, "a")
    if file then
        file:write(line .. "\n")
        file:close()
    else
        print("can not write")
    end
    sleep(0.1)  -- 10Hz
end