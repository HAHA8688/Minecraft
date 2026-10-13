-- sensor_reader.lua
-- 传感器读取和信息初步整合
-- 默认机头朝向为“+x”, 机腹朝向为“+z”, 机体右侧为“+y”；FRD约定
-- 由于传感器限制，偏航角度由角速度积分得到，以启动时的指向为零点（非绝对角度）
-- 适用于 CC:Tweaked 环境
local configOp = require("config_op")
local mathUtils = require("math_utils")
local clamp = mathUtils.clamp
local mix = mathUtils.mix
local eulerToQuat = mathUtils.eulerToQuat
local axisToQuat = mathUtils.axisToQuat
local quatMul = mathUtils.quatMul
local quatRot = mathUtils.quatRot
local quatDot = mathUtils.quatDot
local quatNeg = mathUtils.quatNeg
local quatNorm = mathUtils.quatNorm
local atan = math.atan
local sin = math.sin
local cos = math.cos
local abs = math.abs
-- 数学常量
local pi = math.pi
local half_pi = pi * 0.5
local two_pi = pi * 2


local sensorReader = {
    air_pressure = 0,
    height = 0,
    yaw_rate = 0,
    pit_rate = 0,
    rol_rate = 0,
    last_pit_rate = 0,
    last_rol_rate = 0,
    last_yaw_rate = 0,
    last_gb_pit = 0,
    last_gb_rol = 0,
    yaw = 0,
    pitch = 0,
    roll = 0,
    yaw_quat = {1, 0, 0, 0},
    quaternion = {1, 0, 0, 0},
    x_acc = 0,
    y_acc = 0,
    z_acc = 0,
    x_speed = 0,
    y_speed = 0,
    z_speed = 0,
    last_x_speed = 0,
    last_y_speed = 0,
    last_z_speed = 0,
    last_raw_x_speed = 0,
    last_raw_y_speed = 0,
    last_raw_z_speed = 0,
    gx_acc = 0,
    gy_acc = 0,
    gz_acc = 0,
    gx_speed = 0,
    gy_speed = 0,
    gz_speed = 0,
    grav_vec = {0, 0, 0}
}
sensorReader.__index = sensorReader


local default_settings = {
    AltitudeSensor = {},
    GimbalSensor = {
        dir = 0            -- dir = 0, 1, 2, 3 分别对应0度，90度，180度和270度
    },
    VelocitySensors = {    -- dir需要设置为1或-1，代表传感器极性
        XVeSensorInZp = {
            dir = 0,
            name = ""
        },
        XVeSensorInZn = {
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
        }
    },
    x_sensors_dis = 0,     -- 读取x轴速度的传感器距离
    y_sensors_dis = 0,     -- 读取y轴速度的传感器距离
    z_sensors_dis = 0,     -- 读取z轴速度的传感器距离
}

-- 传感器初始化
function sensorReader.new(settings)
    if not configOp.checkArgs(settings, default_settings) then error("<SensorReader> : Illegal Configuration") end

    local x_sensors_dis = settings.x_sensors_dis
    local y_sensors_dis = settings.y_sensors_dis
    local z_sensors_dis = settings.z_sensors_dis

    if x_sensors_dis <= 0 or y_sensors_dis <= 0 or z_sensors_dis <= 0 then error("<SensorReader> : Illegal Distance of Velocity Sensors") end

    local cali_quat= {}
    eulerToQuat(settings.GimbalSensor.dir * half_pi, 0, 0, cali_quat)    -- 生成将传感器坐标系变化到机体坐标系的四元数
    local T = {
        last_time = os.clock(),
        one_over_x_sensors_dis = 1 / x_sensors_dis,
        one_over_y_sensors_dis = 1 / y_sensors_dis,
        one_over_z_sensors_dis = 1 / z_sensors_dis,
        speed_lowpass_alpha = settings.speed_lowpass_alpha or 0.95,
        angle_rate_lowpass_alpha = settings.angle_rate_lowpass_alpha or 0.95,
        cali_quat = cali_quat,
        grav_vec = {0, 0, 0},
        quaternion = {1, 0, 0, 0},
        yaw_quat = {1, 0, 0, 0},
    }
    setmetatable(T, sensorReader)

    -- 获取外设
    T.GimbalSensor = configOp.getPeripheral("gimbal_sensor", settings.GimbalSensor.name)
    T.AltitudeSensor = configOp.getPeripheral("altitude_sensor", settings.AltitudeSensor.name)
    if not T.GimbalSensor then error("<SensorReader> : Missing Peripheral -- \"gimbal_sensor\"") end
    if not T.AltitudeSensor then error("<SensorReader> : Missing Peripheral -- \"altitude_sensor\"") end

    for key, value in pairs(settings.VelocitySensors) do
        T[key] = configOp.getPeripheral("velocity_sensor", value.name)
        T[key.."Dir"] = value.dir
        if not T[key] then error("<SensorReader> : Missing Peripheral -- \""..value.name.."\"") end
    end
    settings = nil
    return T
end


-- 共享的中间变量
local acc = {0, 0, 0, 0}
local speed = {0, 0, 0, 0}
local pit_rol_quat = {1, 0, 0, 0}
local yaw_quat = {1, 0, 0, 0}
local delta_quat = {1, 0, 0, 0}
local title_quat = {1, 0, 0, 0}

-- 传感器刷新方法
function sensorReader:update()
    -- === dt的计算和处理 ===
    -- 计算dt
    local now = os.clock()
    local dt = now - self.last_time
    -- d非正数dt保护
    if dt <= 0 then dt = 0.0001 end
    self.dt, self.last_time = dt, now
    -- 缓存1/dt
    local one_over_dt = 1 / dt
    local quat = self.quaternion

    -- === 保存上一帧读数 ===
    self.last_yaw_rate, self.last_pit_rate, self.last_rol_rate = self.yaw_rate, self.pit_rate, self.rol_rate
    self.last_x_speed, self.last_y_speed, self.last_z_speed = self.x_speed, self.y_speed, self.z_speed

    -- === 读传感器 ===
    self.height = self.AltitudeSensor.getHeight()
    self.air_pressure = self.AltitudeSensor.getAirPressure()
    local x_speed_zp = self.XVeSensorInZp.getVelocity() * self.XVeSensorInZpDir
    local x_speed_zn = self.XVeSensorInZn.getVelocity() * self.XVeSensorInZnDir
    local y_speed_xp = self.YVeSensorInXp.getVelocity() * self.YVeSensorInXpDir
    local y_speed_xn = self.YVeSensorInXn.getVelocity() * self.YVeSensorInXnDir
    local z_speed_yp = self.ZVeSensorInYp.getVelocity() * self.ZVeSensorInYpDir
    local z_speed_yn = self.ZVeSensorInYn.getVelocity() * self.ZVeSensorInYnDir
    local angles = self.GimbalSensor.getAnglesRad()

    -- === 传感器数据处理 ===
    -- 处理姿态传感器的角度问题
    local gb_pit, gb_rol = angles[1], angles[2]
    angles = nil
    -- 注意, 航空学的姿态传感器返回俯p和横滚r; p, r ∈ (-PI, PI), 0对应双状态，根据上一帧区分
    if gb_pit == 0 then
        local best_distance = two_pi
        for multiplier  = -1, 1 do
            local candidate_angle = pi * multiplier
            local distance = abs(candidate_angle - self.last_gb_pit)
            if distance < best_distance then
                best_distance = distance
                gb_pit = candidate_angle
            end
        end
    end
    if gb_rol == 0 then
        local best_distance = two_pi
        for multiplier  = -1, 1 do
            local candidate_angle = pi * multiplier
            local distance = abs(candidate_angle - self.last_gb_rol)
            if distance < best_distance then
                best_distance = distance
                gb_rol = candidate_angle
            end
        end
    end
    self.last_gb_pit, self.last_gb_rol = gb_pit, gb_rol
    -- 计算重力在局部坐标系的投影向量
    local cos_pit = cos(gb_pit)
    self.grav_vec[1], self.grav_vec[2], self.grav_vec[3] = -sin(gb_pit), cos_pit * sin(gb_rol), cos_pit * cos(gb_rol)

    -- 更新速度
    speed[1] = 0
    speed[2] = (x_speed_zp + x_speed_zn) * 0.5
    speed[3] = (y_speed_xp + y_speed_xn) * 0.5
    speed[4] = (z_speed_yp + z_speed_yn) * 0.5
    self.x_speed = speed[2]
    self.y_speed = speed[3]
    self.z_speed = speed[4]
    -- 更新加速度
    acc[1] = 0
    acc[2] = (speed[2] - self.last_raw_x_speed) * one_over_dt
    acc[3] = (speed[3] - self.last_raw_y_speed) * one_over_dt
    acc[4] = (speed[4] - self.last_raw_z_speed) * one_over_dt
    self.x_acc = acc[2]
    self.y_acc = acc[3]
    self.z_acc = acc[4]
    -- 记录没有滤波的原始数据
    self.last_raw_x_speed = speed[2]
    self.last_raw_y_speed = speed[3]
    self.last_raw_z_speed = speed[4]
    -- 传入false参数，从机体系到世界系
    -- 用于计算世界系下的速度
    quatRot(quat, speed, speed, false)
    self.gx_speed = speed[2]
    self.gy_speed = speed[3]
    self.gz_speed = speed[4]
    --quatRot(quat, acc, acc, false)
    --self.gx_acc = acc[2]
    --self.gy_acc = acc[3]
    --self.gz_acc = acc[4]

    -- 计算角速率
    self.yaw_rate = (y_speed_xp - y_speed_xn) * self.one_over_y_sensors_dis
    self.pit_rate = (x_speed_zp - x_speed_zn) * self.one_over_x_sensors_dis
    self.rol_rate = (z_speed_yp - z_speed_yn) * self.one_over_z_sensors_dis

    -- 计算姿态积分
    local wx, wy, wz = self.rol_rate, self.pit_rate, self.yaw_rate
    local angle_rate_len = math.sqrt(wx * wx + wy * wy + wz * wz)
    if angle_rate_len < 1e-12 then
        delta_quat[1], delta_quat[2], delta_quat[3], delta_quat[4] = 1, 0, 0, 0
    else
        local one_over_len = 1 / angle_rate_len
        axisToQuat(wx * one_over_len, wy * one_over_len, wz * one_over_len, angle_rate_len * dt, delta_quat)
        quatMul(quat, delta_quat, quat)
        quatNorm(quat, quat)
    end

    -- 提取偏航四元数
    local yaw = atan(
        2 * (quat[1] * quat[4] + quat[2] * quat[3]),
        1 - 2 * (quat[3] * quat[3] + quat[4] * quat[4])
    )
    eulerToQuat(yaw, 0, 0, yaw_quat)
    if quatDot(yaw_quat, quat) < 0 then quatNeg(yaw_quat, yaw_quat) end

    -- 积分姿态和实际姿态融合
    local angle_meas = math.sqrt(gb_pit * gb_pit + gb_rol * gb_rol)
    if angle_meas < 1.4 then
        eulerToQuat(0, gb_pit, gb_rol, pit_rol_quat)
        -- 纠正传感器安装偏差
        quatRot(self.cali_quat, pit_rol_quat, pit_rol_quat, false)  -- 传入false参数，从机体系到世界系
        --[[
        G: 世界系
        B: 机体系
        F: 水平系 (由 G 绕 世界Z 旋转当前偏航角φ得到)

        恒等式: Qy * Qpr = (Qyaw * Qpr * Qyaw^-1) * Qy
                  ↑                 ↑               ↑
               化简结果           相似变换        叠加偏航

        值得注意的是，F相对于G旋转了φ，故同样的旋转，在G中必
        须经过Qyaw的相似变化才与在F中等价
        ]]
        quatMul(yaw_quat, pit_rol_quat, title_quat)
        quatNorm(title_quat, title_quat)

        if quatDot(title_quat, quat) < 0 then quatNeg(title_quat, title_quat) end
        -- 计算融合系数
        local alpha = clamp((1.4 - angle_meas) * 0.74, 0, 1)   -- 也就是(1.4 - angle_meas)/(1.4 - 0.05)的近似
        -- 姿态融合
        quat[1] = mix(quat[1], title_quat[1], alpha)
        quat[2] = mix(quat[2], title_quat[2], alpha)
        quat[3] = mix(quat[3], title_quat[3], alpha)
        -- quat[4] = mix(quat[4], title_quat[4], alpha)
        -- 因z分量本质上仅来自积分器，再次混合造成噪声的自激励现象，产生严重的偏航漂移，因此舍去
        quatNorm(quat, quat)
    end

    -- 更新数据
    self.yaw_quat[1] = yaw_quat[1]
    self.yaw_quat[2] = yaw_quat[2]
    self.yaw_quat[3] = yaw_quat[3]
    self.yaw_quat[4] = yaw_quat[4]
    self.yaw = yaw
    self.pitch = atan(
        2 * (quat[1] * quat[3] - quat[4] * quat[2]),
        1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3])
    )
    self.roll = atan(
        2 * (quat[1] * quat[2] + quat[3] * quat[4]),   -- 2*(w*x + y*z)
        1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]) -- 1 - 2*(x² + y²)
    )

    -- 低通滤波
    self.yaw_rate = mix(self.last_yaw_rate, self.yaw_rate, self.angle_rate_lowpass_alpha)
    self.pit_rate = mix(self.last_pit_rate, self.pit_rate, self.angle_rate_lowpass_alpha)
    self.rol_rate = mix(self.last_rol_rate, self.rol_rate, self.angle_rate_lowpass_alpha)
    self.x_speed = mix(self.last_x_speed, self.x_speed, self.speed_lowpass_alpha)
    self.y_speed = mix(self.last_y_speed, self.y_speed, self.speed_lowpass_alpha)
    self.z_speed = mix(self.last_z_speed, self.z_speed, self.speed_lowpass_alpha)

    -- -- ==== 临时调试日志（每帧写入） ====
    -- local file = fs.open("debug_log.txt", "a")
    -- if file then
    --     local w, x, y, z = quat[1], quat[2], quat[3], quat[4]
    --     local line = string.format("(%f, %f, %f); (%f, %f, %f, %f)",
    --         self.yaw_rate, self.pit_rate, self.rol_rate,
    --         w, x, y, z)
    --     file.writeLine(line)
    --     file.close()
    -- end
    -- -- === 调试结束 ===

end

return sensorReader