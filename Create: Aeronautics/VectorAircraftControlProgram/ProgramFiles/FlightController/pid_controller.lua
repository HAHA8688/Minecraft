local PIDController = {
    dt = 1,
    kp = 1.0, ki = 0.5, kd = 0.3,     -- PID参数
    max_output = 1, min_output = -1,  -- 输出限幅
    max_i = 1, min_i = -1,            -- 积分限幅
    separation_i = 0,                 -- 积分分离阈值，大于将分离积分，"0"表示永不分离
    integral = 0,                     -- 积分数据
    is_separation = false,            -- 积分是否被分离
    alpha_d = 0.8,                    -- 微分低通滤波系数
    last_d = 0,                       -- 上一次微分数据
    last_error = 0,                   -- 上一次误差
    last_last_error = 0,              -- 上上次误差
    last_correction = 0,              -- 上一次修正值
    is_increment = false              -- 是否为增量式PID
}


PIDController.__index = PIDController


function PIDController.new(args)

    if args == nil then args = {} end                -- 如无初始化参数，则使用默认值
    
    local NewController = {}

    for key, value in pairs(args) do
        if value ~= nil then NewController[key] = value end
    end
    if NewController.dt <= 0 then NewController.dt = 1 end

    setmetatable(NewController, PIDController)       -- 设置元表
    return NewController
end


function PIDController:update(error, dt)    -- 增量模式不维护具体控制量
    local correction
    local differential
    dt = dt or self.dt
    local one_over_dt = 1 / dt

    if self.is_increment then
        differential = (error - 2 * self.last_error + self.last_last_error) * self.alpha_d + self.last_d * (1 - self.alpha_d)
        correction = self.kp * (error - self.last_error) + self.ki * error * dt + self.kd * differential * one_over_dt
        self.last_d = differential
        self.last_last_error = self.last_error
        self.last_error = error
    else
        differential = (error - self.last_error) * self.alpha_d + self.last_d * (1 - self.alpha_d)   -- 计算微分并且滤波

        self.last_d = differential    -- 更新参数
        self.last_error = error

        if self.separation_i == 0 or math.abs(error) <= self.separation_i then
            self.integral = self.integral + error * dt                              -- 不满足积分分离时，累加积分
            self.is_separation = false
        else
            self.integral = self.integral * 0.99
            self.is_separation = true
        end

        if self.integral > self.max_i then
            self.integral = self.max_i
        elseif self.integral < self.min_i then
            self.integral = self.min_i
        end

        if self.is_separation then
            correction = self.kp * error + self.kd * differential * one_over_dt
        else
            correction = self.kp * error + self.ki * self.integral + self.kd * differential * one_over_dt
        end
        
    end

    if correction > self.max_output then
        correction = self.max_output
    elseif correction < self.min_output then
        correction = self.min_output
    end

    self.last_correction = correction
    return correction
end


-- 一些动态操作方法
function PIDController:reset()

    self.last_error = 0
    self.last_last_error = 0
    self.last_d = 0
    self.integral = 0
    self.last_correction = 0

end


function PIDController:setPIDargs(kp, ki, kd)
    self.kp = kp
    self.ki = ki
    self.kd = kd
end

function PIDController:setDt(dt)
    if dt <= 0 then return end
    self.dt = dt
end


function PIDController:setAlpha(alpha_d)
    self.alpha_d = alpha_d
end


function PIDController:setOutputRange(min_output, max_output)
    if max_output > min_output then
        self.min_output = min_output
        self.max_output = max_output
    end
end


function PIDController:setIntegralRange(min_i, max_i)
    if max_i > min_i then
        self.min_i = min_i
        self.max_i = max_i
    end
end

function PIDController:getLastCorrection()
    return self.last_correction
end

return PIDController