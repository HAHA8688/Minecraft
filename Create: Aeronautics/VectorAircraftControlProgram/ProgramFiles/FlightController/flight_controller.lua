-- flight_controller.lua
-- 这是实现飞行器控制的主程序
-- 适用于 CC:Tweaked 环境
package.path = package.path .. ";/ProgramFiles/?.lua"
local configOp = require("config_op")
local mathUtils = require("math_utils")
local PIDController = require("pid_controller")
local sensorReader = require("sensor_reader")
local controlAllocator = require("control_allocator")

local deadband = mathUtils.deadband
local clamp = mathUtils.clamp
local mix = mathUtils.mix
--local circleClamp = mathUtils.circleClamp
local quatMul = mathUtils.quatMul
-- local eulerToQuat = mathUtils.eulerToQuat
local quatConj = mathUtils.quatConj
local quatNeg = mathUtils.quatNeg
local quatRot = mathUtils.quatRot
local quatDot = mathUtils.quatDot
local quatNorm = mathUtils.quatNorm
-- local floor = math.floor
local abs = math.abs
local sqrt = math.sqrt

-- ##配置处理##

-- 加载配置
local config = configOp.load(shell.resolve("./config.cfg"))

-- 默认配置
local default_config = {
    sleep_time = 0.05,
    height_deadband = 0.5,
    angle_err_deadband = 0.006,
    angle_rate_err_deadband = 0.03,
    speed_err_deadband = 0.3,
    pre_grav_feed = 0,
    auto_grav_feed_sens = 0.01,
    auto_grav_feed_deadband = 0.7,
    yaw_rate_damping_kp = 70,
    pit_rate_damping_kp = 70,
    rol_rate_damping_kp = 70,
    thro_sens = 0.2,
    atti_sens = 0.05,
    pit_static_ff_sens = 0.005,
    pit_static_ff_max = 25,
    rol_static_ff_sens = 0.005,
    rol_static_ff_max = 25,
    pit_z_acc_ff_sens = 0.003,
    pit_z_acc_ff_max = 25,
    rol_z_acc_ff_sens = 0.003,
    rol_z_acc_ff_max = 25,
    pit_x_acc_ff_sens = 0.003,
    pit_x_acc_ff_max = 25,
    rol_y_acc_ff_sens = 0.003,
    rol_y_acc_ff_max = 25,
    x_sp_ctrl_ff_max = 100,
    y_sp_ctrl_ff_max = 80,
    z_sp_ctrl_ff_max = 100,
    sim_resistance = 0.01,
    modem_side = "back",
    sensor_data_protocol = "SensorData",
    heartbeat_protocol = "FlightHeartbeat",
    exit_protocol = "FlightExit",
    LinkedTypewriter = {
        key_codes = {
        key_forward = 87,
        key_back = 83,
        key_left = 65,
        key_right = 68,
        key_up = 32,
        key_down = 340,
        key_turn_left = 81,
        key_turn_right = 69,
        key_pitch_up = 265,
        key_pitch_down = 264,
        key_roll_left = 263,
        key_roll_right = 262,
        key_freeze_atti = 258,
        key_stop = 82,
        key_cruise = 342,
        key_reboot = 259,
        }
    },
    NavigationTable = {},
    RedstoneRelay = {
        name = "",
        throttle_side = "back",
        clutch_side = "left"
    },
    Actuators = {
        DefaultMethod = "",
        max_output = 0,
        min_output = -0,
        int_output = false,
        ActuatorList = {
            -- {name = "", method = ""},
        },
        PseudoInverse = {}
    },
    Sensors = {
        AltitudeSensor = {},
        GimbalSensor = {
            dir = 0
        },
        VelocitySensors = {
            XVeSensorInYp = {
                dir = 0,
                name = ""
            },
            XVeSensorInYn = {
                dir = 0,
                name = ""
            },
            YVeSensorInXp = {
                dir = 0,
                name = ""
            },
            YVeSensorInXn = {
                dir = 0,
                name = ""
            },
            ZVeSensorInYp = {
                dir = 0,
                name = ""
            },
            ZVeSensorInYn = {
                dir = 0,
                name = ""
            },
        },
        x_sensors_dis = 0,
        y_sensors_dis = 0,
        z_sensors_dis = 0,
    },
    PIDControllers = {
        heightPidCtrl = {
            dt = 1,
            kp = 4, ki = 0.3, kd = 0.1,
            max_output = 128, min_output = -128,
            max_i = 64, min_i = -64,
            separation_i = 30,
            alpha_d = 0.5,
            is_increment = false
        },
        yawAttiPidCtrl = {
            dt = 1,
            kp = 12, ki = 2, kd = 3,
            max_output = 10, min_output = -10,
            max_i = 25, min_i = -25,
            separation_i = 0.5,
            alpha_d = 0.8,
            is_increment = false
        },
        pitAttiPidCtrl = {
            dt = 1,
            kp = 12, ki = 2, kd = 3.5,
            max_output = 10, min_output = -10,
            max_i = 25, min_i = -25,
            separation_i = 0.5,
            alpha_d = 0.8,
            is_increment = false
        },
        rolAttiPidCtrl = {
            dt = 1,
            kp = 12, ki = 2, kd = 3.5,
            max_output = 10, min_output = -10,
            max_i = 25, min_i = -25,
            separation_i = 0.5,
            alpha_d = 0.8,
            is_increment = false
        },
        yawRatePidCtrl = {
            dt = 1,
            kp = 8, ki = 1, kd = 3,
            max_output = 200, min_output = -200,
            max_i = 15, min_i = -15,
            separation_i = 0.3,
            alpha_d = 0.9,
            is_increment = false
        },
        pitRatePidCtrl = {
            dt = 1,
            kp = 10, ki = 1, kd = 4,
            max_output = 200, min_output = -200,
            max_i = 15, min_i = -15,
            separation_i = 0.3,
            alpha_d = 0.9,
            is_increment = false
        },
        rolRatePidCtrl = {
            dt = 1,
            kp = 10, ki = 1, kd = 4,
            max_output = 200, min_output = -200,
            max_i = 15, min_i = -15,
            separation_i = 0.3,
            alpha_d = 0.9,
            is_increment = false
        },
        gxSpeedPidCtrl = {
            dt = 1,
            kp = 15, ki = 1, kd = 1,
            max_output = 55, min_output = -55,
            max_i = 3, min_i = -3,
            separation_i = 3,
            alpha_d = 0.7,
            is_increment = false
        },
        gySpeedPidCtrl = {
            dt = 1,
            kp = 15, ki = 1, kd = 1,
            max_output = 55, min_output = -55,
            max_i = 3, min_i = -3,
            separation_i = 0.5,
            alpha_d = 0.6,
            is_increment = false
        },
        gzSpeedPidCtrl = {
            dt = 1,
            kp = 15, ki = 1, kd = 1,
            max_output = 55, min_output = -55,
            max_i = 3, min_i = -3,
            separation_i = 3,
            alpha_d = 0.7,
            is_increment = false
        }
    }
}

-- 合并配置
local settings = configOp.merge(config, default_config)
local pidSettings = settings.PIDControllers

-- 提取配置
local sleep_time = settings.sleep_time
local height_deadband = settings.height_deadband
local angle_err_deadband = settings.angle_err_deadband
local angle_rate_err_deadband = settings.angle_rate_err_deadband
local speed_err_deadband = settings.speed_err_deadband
local pre_grav_feed = settings.pre_grav_feed
local auto_grav_feed_sens = settings.auto_grav_feed_sens
local auto_grav_feed_deadband = settings.auto_grav_feed_deadband
local yaw_rate_damping_kp = settings.yaw_rate_damping_kp
local pit_rate_damping_kp = settings.pit_rate_damping_kp
local rol_rate_damping_kp = settings.rol_rate_damping_kp
local thro_sens = settings.thro_sens
local atti_sens = settings.atti_sens
local pit_static_ff_sens = settings.pit_static_ff_sens
local pit_static_ff_max = settings.pit_static_ff_max
local rol_static_ff_sens = settings.rol_static_ff_sens
local rol_static_ff_max = settings.rol_static_ff_max
local pit_z_acc_ff_sens = settings.pit_z_acc_ff_sens
local pit_z_acc_ff_max = settings.pit_z_acc_ff_max
local rol_z_acc_ff_sens = settings.rol_z_acc_ff_sens
local rol_z_acc_ff_max = settings.rol_z_acc_ff_max
local pit_x_acc_ff_sens = settings.pit_x_acc_ff_sens
local pit_x_acc_ff_max = settings.pit_x_acc_ff_max
local rol_y_acc_ff_sens = settings.rol_y_acc_ff_sens
local rol_y_acc_ff_max = settings.rol_y_acc_ff_max
local x_sp_ctrl_ff_max = settings.x_sp_ctrl_ff_max
local y_sp_ctrl_ff_max = settings.y_sp_ctrl_ff_max
local z_sp_ctrl_ff_max = settings.z_sp_ctrl_ff_max
local sim_resistance = settings.sim_resistance
local gear_pos_input_side = settings.RedstoneRelay.throttle_side
local clutch_input_side = settings.RedstoneRelay.clutch_side
local modem_side = settings.modem_side
local sensor_data_protocol = settings.sensor_data_protocol
local heartbeat_protocol = settings.heartbeat_protocol
local exit_protocol = settings.exit_protocol
local key_codes = settings.LinkedTypewriter.key_codes

-- 实例化传感器和分配器模块
local sr = sensorReader.new(settings.Sensors)
local ca = controlAllocator.new(settings.Actuators)
if not (sr and ca) then return end

-- 获取外设
local LiTypewriter = configOp.getPeripheral("linked_typewriter", settings.LinkedTypewriter.name)
--local NavigationTable = configOp.getPeripheral("navigation_table", settings.LinkedTypewriter.name)
local RedstoneRelay = configOp.getPeripheral("redstone_relay", settings.RedstoneRelay.name)
if not LiTypewriter then error("<FlightController> : Missing Peripheral -- \"linked_typewriter\"") end
if not RedstoneRelay then error("<FlightController> : Missing Peripheral -- \"redstone_relay\"") end

-- 实例化PID控制器
local heightPidCtrl = PIDController.new(pidSettings.heightPidCtrl)
local yawAttiPidCtrl = PIDController.new(pidSettings.yawAttiPidCtrl)
local pitAttiPidCtrl = PIDController.new(pidSettings.pitAttiPidCtrl)
local rolAttiPidCtrl = PIDController.new(pidSettings.rolAttiPidCtrl)
local yawRatePidCtrl = PIDController.new(pidSettings.yawRatePidCtrl)
local pitRatePidCtrl = PIDController.new(pidSettings.pitRatePidCtrl)
local rolRatePidCtrl = PIDController.new(pidSettings.rolRatePidCtrl)
local gxSpeedPidCtrl = PIDController.new(pidSettings.gxSpeedPidCtrl)
local gySpeedPidCtrl = PIDController.new(pidSettings.gySpeedPidCtrl)
local gzSpeedPidCtrl = PIDController.new(pidSettings.gzSpeedPidCtrl)

-- 释放配置文件
config = nil
default_config = nil
settings = nil
pidSettings = nil

-- ##网络通信##
rednet.open(modem_side)
local time_stamp = os.epoch("utc")
local local_id = os.getComputerID()
local last_heartbeat_time = 0
local heartbeat_interval = 1
local sensor_data_package = {0, 0, 0, 0, 0, 0, 0}
local function rednetReceive()
    while true do
        sleep(0.5)
        local _, message, _ = rednet.receive(exit_protocol, nil)
        if message == time_stamp then return end
    end
end
local function rednetSend()
    while true do
        sleep(0.05)
        -- 更新数据包
        sensor_data_package[1] = sr.pitch
        sensor_data_package[2] = sr.roll
        sensor_data_package[3] = sr.x_speed
        sensor_data_package[4] = sr.y_speed
        sensor_data_package[5] = sr.z_speed
        sensor_data_package[6] = sr.height
        sensor_data_package[7] = sr.air_pressure
        rednet.send(local_id, sensor_data_package, sensor_data_protocol)
        -- 发送心跳
        local now = os.clock()
        if now - last_heartbeat_time > heartbeat_interval then
            rednet.send(local_id, time_stamp, heartbeat_protocol)
            last_heartbeat_time = now
        end
    end
end

-- ##引擎通信##
local engines_update = false
local ca_input = {0, 0, 0, 0, 0, 0}
local function enginesUpdate()
    while true do
        sleep(0.05)
        if engines_update then
            engines_update = false
            ca:update(ca_input)
        end
    end
end

-- ##飞控逻辑##
local function main()
    -- 传感器和姿态变量
    sr:update()
    local tar_height = sr.height
    local cur_quat = sr.quaternion
    local conj_cur_quat = {}
    local tar_quat = {1, 0, 0, 0}
    local err_quat = {1, 0, 0, 0}
    local tar_yaw_rate, tar_rol_rate, tar_pit_rate
    local x_ff, y_ff, z_ff
    local pit_static_ff, pit_x_acc_ff, pit_z_acc_ff = 0, 0, 0
    local rol_static_ff, rol_y_acc_ff, rol_z_acc_ff = 0, 0, 0
    local tar_g_speed = {0, sr.gx_speed, sr.gy_speed, sr.gz_speed}
    local tar_speed = {0, 0, 0, 0}
    local g_speed_corr = {0, 0, 0, 0}
    -- 控制器变量
    local keyTable = {}
    local getPressedKeyCodes = LiTypewriter.getPressedKeyCodes
    local getAnalogInput = RedstoneRelay.getAnalogInput
    local getInput = RedstoneRelay.getInput
    local last_gear_pos = 0
    local is_neutral_gear = false
    local last_clutch_pos = true
    local is_cruise_ctrl = false
    local yaw_brake, pit_brake, rol_brake = false, false, false
    -- 按键处理函数
    local function keyCodesProcess(input, output)
        for key, value in pairs(output) do
            if value > 0 then output[key] = -value
            elseif value < 0 then output[key] = nil end
        end
        for _, code in ipairs(input) do
            local output_value = output[code]
            if output_value then output[code] = abs(output_value) + 1
            else output[code] = 1 end
        end
    end
    -- 主循环
    while true do
        sleep(sleep_time)
        -- 刷新传感器
        sr:update()
        local dt = sr.dt
        -- 姿态解算
        local dot = quatDot(tar_quat, cur_quat)
        if dot < 0 then quatNeg(tar_quat, tar_quat) end  -- 取反目标四元数，避免长路径
        quatConj(cur_quat, conj_cur_quat)
        quatMul(conj_cur_quat, tar_quat, err_quat)
        -- 姿态环PID计算
        tar_pit_rate = pitAttiPidCtrl:update(deadband(err_quat[3], angle_err_deadband), dt)
        tar_rol_rate = rolAttiPidCtrl:update(deadband(err_quat[2], angle_err_deadband), dt)
        tar_yaw_rate = yawAttiPidCtrl:update(deadband(err_quat[4], angle_err_deadband), dt)
        -- 离合器状态处理
        local clutch_pos = getInput(clutch_input_side)
        if clutch_pos ~= last_clutch_pos then
            last_clutch_pos = clutch_pos
            heightPidCtrl:reset()
            yawAttiPidCtrl:reset()
            pitAttiPidCtrl:reset()
            rolAttiPidCtrl:reset()
            yawRatePidCtrl:reset()
            pitRatePidCtrl:reset()
            rolRatePidCtrl:reset()
            gxSpeedPidCtrl:reset()
            gySpeedPidCtrl:reset()
            gzSpeedPidCtrl:reset()
        end
        if clutch_pos then goto END end    -- 离合被压下时跳过控制

        -- ##按键控制##
        -- 挡位逻辑
        local gear_pos = getAnalogInput(gear_pos_input_side)
        if gear_pos ~= last_gear_pos then
            last_gear_pos = gear_pos
            gxSpeedPidCtrl:reset()
            gySpeedPidCtrl:reset()
            gzSpeedPidCtrl:reset()
            if gear_pos == 0 then
                ca:setOutputRange(-15, 15)
                is_neutral_gear = true
            elseif gear_pos == 1 then
                ca:setOutputRange(-16, 16)
                is_neutral_gear = false
            else
                ca:setOutputRange(-256, 256)
                is_neutral_gear = false
            end
        end
        -- 打字机输入获取和操作逻辑
        keyCodesProcess(getPressedKeyCodes(), keyTable)
        local key_reboot =  keyTable[key_codes.key_reboot] or 0
        if key_reboot < -9 then os.reboot() end
        -- 非空挡处理控制输入
        if is_neutral_gear then
            x_ff, y_ff, z_ff = 0, 0 , 0
        else
            -- 模式处理
            if next(keyTable) ~= nil then
                -- 判断是否为巡航模式
                is_cruise_ctrl = false
                local key_cruise = keyTable[key_codes.key_cruise] or 0
                if key_cruise > 9 or key_cruise < 0 then
                    is_cruise_ctrl = true
                    tar_quat[1] = sr.yaw_quat[1]
                    tar_quat[2] = sr.yaw_quat[2]
                    tar_quat[3] = sr.yaw_quat[3]
                    tar_quat[4] = sr.yaw_quat[4]
                end
                -- 判断是否刹车
                local key_stop =  keyTable[key_codes.key_stop] or 0
                if key_stop > 0 then
                    tar_g_speed[2], tar_g_speed[3], tar_g_speed[4] = 0, 0, 0
                    tar_height = sr.height
                    heightPidCtrl:reset()
                end
                -- 判断是否回中
                local key_freeze_atti = keyTable[key_codes.key_freeze_atti] or 0
                if key_freeze_atti > 0 then
                    tar_quat[1] = mix(tar_quat[1], sr.yaw_quat[1], 0.2)
                    tar_quat[2] = mix(tar_quat[2], sr.yaw_quat[2], 0.2)
                    tar_quat[3] = mix(tar_quat[3], sr.yaw_quat[3], 0.2)
                    tar_quat[4] = mix(tar_quat[4], sr.yaw_quat[4], 0.2)
                    quatNorm(tar_quat, tar_quat)
                end
            end

            -- ##速度环按键处理##
            local key_forward = keyTable[key_codes.key_forward] or 0
            local key_back = keyTable[key_codes.key_back] or 0
            local key_right = keyTable[key_codes.key_right] or 0
            local key_left = keyTable[key_codes.key_left] or 0
            local key_down = keyTable[key_codes.key_down] or 0
            local key_up = keyTable[key_codes.key_up] or 0
            -- 速度衰减
            if not is_cruise_ctrl then
                local vgx = tar_g_speed[2]
                local vgy = tar_g_speed[3]
                local vgz = tar_g_speed[4]
                local scale = 1 / (1 + sim_resistance * sqrt(vgx * vgx + vgy * vgy + vgz * vgz) * dt)
                tar_g_speed[2] = vgx * scale
                tar_g_speed[3] = vgy * scale
                tar_g_speed[4] = vgz * scale
            end
            quatRot(cur_quat, tar_g_speed, tar_speed, true) -- 传入true参数，从世界系到机体系
            if key_forward > 0 or key_back > 0 then tar_speed[2] = sr.x_speed end
            if key_right > 0 or key_left > 0 then tar_speed[3] = sr.y_speed end
            if key_down > 0 or key_up > 0 then tar_speed[4] = sr.z_speed end
            quatRot(cur_quat, tar_speed, tar_g_speed, false)  -- 传入false参数，从机体系到世界系
            -- 力控制逻辑
            if key_forward < 0 then key_forward = 0 end
            if key_back < 0 then key_back = 0 end
            if key_right < 0 then key_right = 0 end
            if key_left < 0 then key_left = 0 end
            if key_down < 0 then key_down = 0 end
            if key_up < 0 then key_up = 0 end
            local sp_ctrl_ff_sens = thro_sens * gear_pos * 0.066666667    -- 乘0.066666667也就是除15
            x_ff = mix(0, x_sp_ctrl_ff_max, clamp((key_forward - key_back) * sp_ctrl_ff_sens, -1, 1))
            y_ff = mix(0, y_sp_ctrl_ff_max, clamp((key_right - key_left) * sp_ctrl_ff_sens, -1, 1))
            z_ff = mix(0, z_sp_ctrl_ff_max, clamp((key_down - key_up) * sp_ctrl_ff_sens, -1, 1))

            -- ##角速度环按键处理##
            -- 偏航
            local key_turn_left = keyTable[key_codes.key_turn_left] or 0
            local key_turn_right = keyTable[key_codes.key_turn_right] or 0
            if (key_turn_left < 0 or key_turn_right < 0) or (key_turn_left > 0 and key_turn_right > 0) then
                tar_yaw_rate = 0
                yawAttiPidCtrl:reset()
                yaw_brake = true
            elseif key_turn_left > 0 then
                tar_yaw_rate = mix(0, -25, clamp(key_turn_left * atti_sens, 0, 1))
            elseif key_turn_right > 0 then
                tar_yaw_rate = mix(0, 25, clamp(key_turn_right * atti_sens, 0, 1))
            end
            -- 俯仰
            local key_pitch_up = keyTable[key_codes.key_pitch_up] or 0
            local key_pitch_down = keyTable[key_codes.key_pitch_down] or 0
            if (key_pitch_up < 0 or key_pitch_down < 0) or (key_pitch_up > 0 and key_pitch_down > 0) then
                tar_pit_rate = 0
                pitAttiPidCtrl:reset()
                pit_brake = true
            elseif key_pitch_up > 0 then
                tar_pit_rate = mix(0, 45, clamp(key_pitch_up * atti_sens, 0, 1))
            elseif key_pitch_down > 0 then
                tar_pit_rate = mix(0, -45, clamp(key_pitch_down * atti_sens, 0, 1))
            end
            -- 横滚
            local key_roll_right = keyTable[key_codes.key_roll_right] or 0
            local key_roll_left = keyTable[key_codes.key_roll_left] or 0
            if (key_roll_right < 0 or key_roll_left < 0) or (key_roll_right > 0 and key_roll_left > 0) then
                tar_rol_rate = 0
                rolAttiPidCtrl:reset()
                rol_brake = true
            elseif key_roll_right > 0 then
                tar_rol_rate = mix(0, 45, clamp(key_roll_right * atti_sens, 0, 1))
            elseif key_roll_left > 0 then
                tar_rol_rate = mix(0, -45, clamp(key_roll_left * atti_sens, 0, 1))
            end
        end

        -- ##非姿态环PID计算和前馈##
        local grav_z = sr.grav_vec[3]
        local x_acc, y_acc, z_acc = sr.x_acc, sr.y_acc, sr.z_acc
        local yaw_rate, pit_rate, rol_rate = sr.yaw_rate, sr.pit_rate, sr.rol_rate
        local last_yaw_rate, last_pit_rate, last_rol_rate = sr.last_yaw_rate, sr.last_pit_rate, sr.last_rol_rate
        -- 重力补偿PID
        local grav_feed = clamp(pre_grav_feed + heightPidCtrl:update(deadband(tar_height - sr.height, height_deadband), dt), -200, 200)
        if abs(tar_g_speed[4]) > auto_grav_feed_deadband then
            tar_height = sr.height - tar_g_speed[4] * dt
            heightPidCtrl:reset()
        else
            pre_grav_feed = mix(pre_grav_feed, grav_feed, auto_grav_feed_sens)
        end
        local grav_x_ff = -grav_feed * sr.grav_vec[1]
        local grav_y_ff = -grav_feed * sr.grav_vec[2]
        local grav_z_ff = -grav_feed * grav_z
        -- 其他PID
        local yaw_pid = yawRatePidCtrl:update(deadband(tar_yaw_rate - yaw_rate, angle_rate_err_deadband), dt)
        local pit_pid = pitRatePidCtrl:update(deadband(tar_pit_rate - pit_rate, angle_rate_err_deadband), dt)
        local rol_pid = rolRatePidCtrl:update(deadband(tar_rol_rate - rol_rate, angle_rate_err_deadband), dt)
        local yaw_corr = yaw_pid
        local pit_corr = pit_pid
        local rol_corr = rol_pid
        g_speed_corr[2] = gxSpeedPidCtrl:update(deadband(tar_g_speed[2] - sr.gx_speed, speed_err_deadband), dt)
        g_speed_corr[3] = gySpeedPidCtrl:update(deadband(tar_g_speed[3] - sr.gy_speed, speed_err_deadband), dt)
        g_speed_corr[4] = gzSpeedPidCtrl:update(deadband(tar_g_speed[4] - sr.gz_speed, speed_err_deadband), dt)
        quatRot(cur_quat, g_speed_corr, g_speed_corr, true)  -- 传入true参数，从世界系到机体系
        local is_stable = true
        if abs(yaw_rate) > 0.1 or abs(pit_rate) > 0.1 or abs(rol_rate) > 0.1 then is_stable = false end
        local pit_static_corr, rol_static_corr, pit_z_acc_corr, rol_z_acc_corr = 0, 0, 0, 0
        -- 悬停时的俯仰横滚补偿前馈
        pit_static_corr, rol_static_corr = pit_static_ff * grav_z, rol_static_ff * grav_z
        pit_corr, rol_corr = pit_corr + pit_static_corr, rol_corr + rol_static_corr
        if abs(grav_z) > 0.01 and abs(sr.x_acc) < 0.2 and abs(sr.y_acc) < 0.2 and abs(sr.z_acc) < 0.2 and is_stable then
            local one_of_grav_z = 1 / grav_z
            if abs(sr.pit_rate) < 0.1 then
                pit_static_ff = clamp(mix(pit_static_ff, pit_corr * one_of_grav_z, pit_static_ff_sens), -pit_static_ff_max, pit_static_ff_max)
            end
            if abs(sr.rol_rate) < 0.1 then
                rol_static_ff = clamp(mix(rol_static_ff, rol_corr * one_of_grav_z, rol_static_ff_sens), -rol_static_ff_max, rol_static_ff_max)
            end
        end
        -- z轴机动时的俯仰横滚补偿前馈
        pit_z_acc_corr, rol_z_acc_corr = pit_z_acc_ff * z_acc, rol_z_acc_ff * z_acc
        pit_corr, rol_corr = pit_corr + pit_z_acc_corr, rol_corr + rol_z_acc_corr
        if abs(z_acc) > 0.2 and is_stable then
            local one_of_z_acc = 1 / z_acc
            pit_z_acc_ff = clamp(mix(pit_z_acc_ff, (pit_corr - pit_static_corr) * one_of_z_acc, pit_z_acc_ff_sens), -pit_z_acc_ff_max, pit_z_acc_ff_max)
            rol_z_acc_ff = clamp(mix(rol_z_acc_ff, (rol_corr - rol_static_corr) * one_of_z_acc, rol_z_acc_ff_sens), -rol_z_acc_ff_max, rol_z_acc_ff_max)
        end
        -- x轴机动时的俯仰补偿前馈
        pit_corr = pit_corr + pit_x_acc_ff * x_acc
        if abs(x_acc) > 0.2 and is_stable then
            pit_x_acc_ff = clamp(mix(pit_x_acc_ff, (pit_corr - pit_static_corr - pit_z_acc_corr) / x_acc, pit_x_acc_ff_sens), -pit_x_acc_ff_max, pit_x_acc_ff_max)
        end
        -- y轴机动时的横滚补偿前馈
        rol_corr = rol_corr + rol_y_acc_ff * y_acc
        if abs(y_acc) > 0.2 and is_stable then
            rol_y_acc_ff = clamp(mix(rol_y_acc_ff, (rol_corr - rol_static_corr - rol_z_acc_corr) / y_acc, rol_y_acc_ff_sens), -rol_y_acc_ff_max, rol_y_acc_ff_max)
        end
        -- 角速度阻尼
        if yaw_brake then
            if yaw_rate == 0 or (yaw_rate > 0) ~= (last_yaw_rate > 0) then
                yaw_brake = false
                tar_quat[1]  = cur_quat[1]
                tar_quat[2]  = cur_quat[2]
                tar_quat[3]  = cur_quat[3]
                tar_quat[4]  = cur_quat[4]
            end
            yaw_corr = yaw_corr - yaw_rate_damping_kp * yaw_rate
        end
        if pit_brake then
            if pit_rate == 0 or (pit_rate > 0) ~= (last_pit_rate > 0) then
                pit_brake = false
                tar_quat[1]  = cur_quat[1]
                tar_quat[2]  = cur_quat[2]
                tar_quat[3]  = cur_quat[3]
                tar_quat[4]  = cur_quat[4]
            end
            pit_corr = pit_corr - pit_rate_damping_kp * pit_rate
        end
        if rol_brake then
            if rol_rate == 0 or (rol_rate > 0) ~= (last_rol_rate > 0) then
                rol_brake = false
                tar_quat[1]  = cur_quat[1]
                tar_quat[2]  = cur_quat[2]
                tar_quat[3]  = cur_quat[3]
                tar_quat[4]  = cur_quat[4]
            end
            rol_corr = rol_corr - rol_rate_damping_kp * rol_rate
        end

        -- ##分配器##
        -- 计算2
        ca_input[1] = rol_corr                              -- tau_roll
        ca_input[2] = pit_corr                              -- tau_pitch
        ca_input[3] = yaw_corr                              -- tau_yaw
        ca_input[4] = g_speed_corr[2] + x_ff + grav_x_ff    -- fx
        ca_input[5] = g_speed_corr[3] + y_ff + grav_y_ff    -- fy
        ca_input[6] = g_speed_corr[4] + z_ff + grav_z_ff    -- fz
        -- 更新标志位
        engines_update = true

        ::END::

        -- ##调试输出##
        --print(pit_static_ff..", "..rol_static_ff)
        --print(grav_feed..", "..pre_grav_feed..", "..tar_height)
        --print(textutils.serialize(U))
        --print(sr.yaw..", "..sr.pitch..", "..sr.roll)
        --print(dt)
        --print(textutils.serialize(U))
        --print(grav_feed)
        --print(textutils.serialize(sr.quaternion))
        --print(sr.y_speed)
        --print(textutils.serialize(tar_quat))
    end
end

parallel.waitForAny(main, rednetReceive, rednetSend, enginesUpdate)