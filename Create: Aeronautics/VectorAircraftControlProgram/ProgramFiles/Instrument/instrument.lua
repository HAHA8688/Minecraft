-- instrument.lua
-- 仪表，通过 rednet 接收飞控数据，绘制地平仪和高度信息

package.path = package.path .. ";/ProgramFiles/?.lua"
local configOp = require("config_op")
local floor = math.floor
local abs = math.abs
local tan = math.tan
local rep = string.rep
local format = string.format
local one_over_pi = 1 / math.pi
local two_over_pi = 2 * one_over_pi
local half_pi = math.pi * 0.5
local function clamp(value, min_value, max_value)
    if value > max_value then return max_value
    elseif value < min_value then return min_value
    else return value end
end
local function strClamp(str, len)
    local str_len = #str
    if str_len > len then return str:sub(1, len)
    else return str .. rep(" ", len - str_len)
    end
end

-- 加载配置文件
local config, err = configOp.load(shell.resolve("./config.cfg"))
if err ~= "SUCCESS" then
    error("Failed to load instruments config: " .. err)
end
-- 默认配置
local defaultConfig = {
    MainMonitor = {
        name = ""
    },
    modem_side = "bottom",
    sensor_data_protocol = "SensorData",
    heartbeat_protocol = "InstrumentHeartbeat",
    exit_protocol = "InstrumentExit",
    inf_area_bc = "9",
    inf_area_tc = "0",
    sky_bc = "3",
    ground_bc = "c",
    text_offset_x = 1,
    text_offset_y = 1
}
-- 合并配置
local settings = configOp.merge(config, defaultConfig)
-- 提取配置
local data_protocol = settings.sensor_data_protocol
local heartbeat_protocol = settings.heartbeat_protocol
local exit_protocol = settings.exit_protocol

rednet.open(settings.modem_side)
local time_stamp = os.epoch("utc")
local local_id = os.getComputerID()
local heartbeat_interval = 1
local last_heartbeat_time = 0
local function rednetReceive()
    while true do
        sleep(0.5)
        local _, message, _ = rednet.receive(exit_protocol, nil)
        if message == time_stamp then return end
    end
end
local function rednetSend()
    -- 心跳
    while true do
        sleep(0.5)
        local now = os.clock()
        if now - last_heartbeat_time >= heartbeat_interval then
            rednet.send(local_id, time_stamp, heartbeat_protocol)
            last_heartbeat_time = now
        end
    end
end


local function main()
    -- 绘图常量
    local inf_area_bc = settings.inf_area_bc
    local inf_area_tc = settings.inf_area_tc
    local sky_bc = settings.sky_bc
    local ground_bc = settings.ground_bc
    -- 获取显示器
    local monitor = configOp.getPeripheral("monitor", settings.MainMonitor.name)
    if not monitor then error("No monitor found") end
    -- 设置缩放
    monitor.setTextScale(0.5)
    -- 显示器尺寸和相关常量
    local m_width, m_height = monitor.getSize()
    --print(m_width, m_height)
    local half_m_height = m_height * 0.5

    local left_width = floor(m_width * 1 / 3)
    if left_width < 1 then left_width = 1 end

    local right_width = m_width - left_width
    local half_right_width = right_width * 0.5

    local left_area_bc_str = rep(inf_area_bc, left_width)
    local tc_str = rep(inf_area_tc, m_width)

    local inf_str_list = {}
    --文本偏移
    local text_offset_y = settings.text_offset_y
    local text_offset_x_str = rep(" ", settings.text_offset_x)
    -- 其他变量
    local rad_2_angle = 57.2958 -- 弧度转化为角度的系数
    local pitch, roll = 0, 0
    local altitude, air_pressure = 0, 0
    local x_speed, y_speed, z_speed = 0, 0, 0

    config = nil
    defaultConfig = nil
    settings = nil

    -- 创建缓冲区
    local output_buffer = {}
    for i = 1, m_height do output_buffer[i] = {"", ""} end

    while true do
        sleep(0.033)
        -- 接收传感器数据
        local _, data, _ = rednet.receive(data_protocol, nil)
        if type(data) == "table" then
            -- 数据格式[pitch, roll, x_speed, y_speed, z_speed, height, air_pressure]
            pitch = data[1]
            roll = data[2]
            x_speed = data[3]
            y_speed = data[4]
            z_speed = data[5]
            altitude = data[6]
            air_pressure = data[7]
        end

        inf_str_list[1] = text_offset_x_str .. format("AP: %5.2f %%", air_pressure * 100)
        inf_str_list[2] = text_offset_x_str .. format("AT: %5.2f m", altitude)
        inf_str_list[3] = text_offset_x_str .. format("VX: %5.2f m/s", x_speed)
        inf_str_list[4] = text_offset_x_str .. format("VY: %5.2f m/s", y_speed)
        inf_str_list[5] = text_offset_x_str .. format("VZ: %5.2f m/s", z_speed)
        inf_str_list[6] = text_offset_x_str .. format("PT: %5.2f deg", pitch * rad_2_angle)
        inf_str_list[7] = text_offset_x_str .. format("RL: %5.2f deg", roll * rad_2_angle)

        local is_upside_down = (abs(pitch) > half_pi) or (abs(roll) > half_pi)

        if abs(roll) < 10E-3 then
            local b = floor((pitch * one_over_pi + 1) * half_m_height + 0.5)
            local bg_color_1, bg_color_2
            if is_upside_down then bg_color_1, bg_color_2 = ground_bc, sky_bc
            else bg_color_1, bg_color_2 = sky_bc, ground_bc end
            for i = 1, m_height do
                local bc_str, inf_str
                local j = i - text_offset_y
                if j > 0 then inf_str = inf_str_list[j] or ""
                else inf_str = "" end
                inf_str = strClamp(inf_str, m_width)
                local old = output_buffer[i]
                if i <= b then bc_str = left_area_bc_str .. rep(bg_color_1, right_width)
                else bc_str = left_area_bc_str .. rep(bg_color_2, right_width) end
                if old[1] ~= bc_str or old[2] ~= inf_str then
                    old[1] = bc_str
                    old[2] = inf_str
                    monitor.setCursorPos(1, i)
                    monitor.blit(inf_str, tc_str, bc_str)
                end
            end
        else
            local m = -1 / tan(roll)
            local b = left_width + half_right_width - (pitch * two_over_pi + 1) * half_m_height * m
            local bg_color_1, bg_color_2
            if (m < 0) == is_upside_down then bg_color_1, bg_color_2 = ground_bc, sky_bc
            else bg_color_1, bg_color_2 = sky_bc, ground_bc end
            for i = 1, m_height do
                local inf_str
                local j = i - text_offset_y
                if j > 0 then inf_str = inf_str_list[j] or ""
                else inf_str = "" end
                inf_str = strClamp(inf_str, m_width)
                local old = output_buffer[i]
                local boundary = clamp(floor(m * i + b + 0.5) - left_width, 0, right_width)
                local bc_str = left_area_bc_str .. rep(bg_color_1, boundary) .. rep(bg_color_2, right_width - boundary)
                if old[1] ~= bc_str or old[2] ~= inf_str then
                    old[1] = bc_str
                    old[2] = inf_str
                    monitor.setCursorPos(1, i)
                    monitor.blit(inf_str, tc_str, bc_str)
                end
            end
        end
    end
end

parallel.waitForAny(main, rednetReceive, rednetSend)