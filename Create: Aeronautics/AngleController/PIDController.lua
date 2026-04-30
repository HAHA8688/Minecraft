local PIDController = {

    kp = 1.0, ki = 0.5, kd = 0.3,     -- PID参数
    dt = 0.05,                        -- 执行周期
    max_i = 1, min_i = -1,            -- 积分限幅
    max_output = 1, min_output = -1,  -- 输出限幅
    alpha_d = 0.8,                    -- 微分低通滤波系数
    last_d = 0,                       -- 上一次微分数据
    integral = 0,                     -- 积分数据
    last_error = 0                    -- 上一次误差

}


function PIDController.new(args)

    if args == nil then args = {} end                -- 如无初始化参数，则使用默认值
    
    local NewController = {}

    local function add(key, value)
        if value == nil then return end
        NewController[key] = value
    end

    for key, value in pairs(args) do                 -- 初始化PID控制器
        add(key, value)
    end

    setmetatable(NewController, PIDController)       -- 设置元表
    PIDController["__index"] = PIDController
    return NewController

end


function PIDController.update(self, error)

    local differential = (error - self.last_error) / self.dt                        -- 计算微分
    differential = differential * self.alpha_d + self.last_d * (1 - self.alpha_d)   -- 微分滤波

    self.last_d = differential    -- 更新参数
    self.last_error = error

    local correction = 0

    if not (self.integral > self.max_i and error > 0) and not (self.integral < self.min_i and error < 0) then
        self.integral = self.integral + error * self.dt    --积分超过限幅且误差同向，不再累加
    end
    
    if self.integral <= self.max_i and self.integral >= self.min_i then
        correction = self.kp * error + self.ki * self.integral + self.kd * differential
    else
        correction = self.kp * error + self.kd * differential
    end

    if correction > self.max_output then
        correction = self.max_output
    elseif correction < self.min_output then
        correction = self.min_output
    end

    return correction

end


function PIDController.reset(self)

    self.last_error = 0
    self.last_d = 0
    self.integral = 0

end


function PIDController.setPIDargs(self, kp, ki, kd)
    self.kp = kp
    self.ki = ki
    self.kd = kd
end


function PIDController.setInterval(self, dt)
    self.dt = dt
end


function PIDController.setAlpha(self, alpha_d)
    self.alpha_d = alpha_d
end


function PIDController.setOutputRange(self, min_output, max_output)
    if max_output > min_output then
        self.min_output = min_output
        self.max_output = max_output
    end
end


function PIDController.setIntegralRange(self, min_i, max_i)
    if max_i > min_i then
        self.min_i = min_i
        self.max_i = max_i
    end
end

return PIDController
