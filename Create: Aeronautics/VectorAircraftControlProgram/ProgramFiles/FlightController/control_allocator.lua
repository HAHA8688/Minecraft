local floor = math.floor

local function actuatorReg(output_buffer, actuator_list, default_method)
    if type(actuator_list) ~= "table" then return {} end
    local actuators = {}
    for i = 1, #actuator_list do
        local actuator_cfg = actuator_list[i]
        if type(actuator_cfg) ~= "table" then return {} end
        local actuator = peripheral.wrap(actuator_cfg.name)
        if not actuator then
            print("<controlAllocator> : Missing Peripheral -- \""..actuator_cfg.name.."\"")
            return {}
        end
        local method = actuator_cfg.method or default_method
        if not method then
            print("<controlAllocator> : Missing Method")
            return {}
        end
        local j = i
        actuators[i] = function() actuator[method](output_buffer[j]) end
    end
    return actuators
end

local controlAllocator = {}
controlAllocator.__index = controlAllocator

function controlAllocator.new(settings)
    local T = {
        int_output = settings.int_output or false,
        output_buffer = {}
    }
    local actuators = actuatorReg(T.output_buffer, settings.ActuatorList, settings.DefaultMethod)
    local actuator_num = #actuators
    local pseudo_inverse = settings.PseudoInverse
    local max_output, min_output = settings.max_output or 0, settings.min_output or 0
    local zero_max_output, zero_min_output = false, false
    if max_output < 0 or min_output > 0 then error("<controlAllocator> : Illegal Output Range") end
    if max_output == 0 then zero_max_output = true end
    if min_output == 0 then zero_min_output = true end
    if type(pseudo_inverse) ~= "table" then error("<controlAllocator> : Illegal Pseudo-Inverse Matrix") end
    if actuator_num == 0 then error("<controlAllocator> : Illegal Actuator List") end
    if actuator_num ~= #pseudo_inverse then error("<controlAllocator> : Illegal Pseudo-Inverse Matrix") end
    for i = 1, actuator_num do
        local row = pseudo_inverse[i]
        if not row or type(row) ~= "table" or #row ~= 6 then
            error("<controlAllocator> : Illegal Pseudo-Inverse Matrix")
        end
    end
    T.pseudo_inverse = pseudo_inverse
    T.actuators = actuators
    T.actuator_num = actuator_num
    T.max_output = max_output
    T.min_output = min_output
    T.zero_max_output = zero_max_output
    T.zero_min_output = zero_min_output
    setmetatable(T, controlAllocator)
    return T
end

function controlAllocator:update(V)
    --print(self.max_output)
    if not V or #V ~= 6 then
        print("<controlAllocator> : Illegal Control Input")
        return
    end
    local pseudo_inverse = self.pseudo_inverse
    local actuator_num = self.actuator_num
    local output_buffer = self.output_buffer
    local max_output, min_output = self.max_output, self.min_output
    local m, n = max_output, -min_output
    for i = 1, actuator_num do
        local row = pseudo_inverse[i]
        local v = row[1] * V[1] + row[2] * V[2] + row[3] * V[3] + row[4] * V[4] + row[5] * V[5] + row[6] * V[6]
        output_buffer[i] = v
        if v > m then m = v
        elseif -v > n then n = -v end
    end
    if self.zero_max_output then m = 0
    else m = max_output / m end
    if self.zero_min_output then n = 0
    else n = -min_output / n end
    if m > n then m = n end
    if m == 0 or m == 1 then
        if self.int_output then
            for i = 1, actuator_num do
                local v = output_buffer[i]
                if v > max_output then v = max_output
                elseif v < min_output then v = min_output end
                output_buffer[i] = floor(v + 0.5)
            end
        else
            for i = 1, actuator_num do
                local v = output_buffer[i]
                if v > max_output then v = max_output
                elseif v < min_output then v = min_output end
                output_buffer[i] = v
            end
        end
    else
        if self.int_output then
            for i = 1, actuator_num do
                local v = output_buffer[i] * m
                if v > max_output then v = max_output
                elseif v < min_output then v = min_output end
                output_buffer[i] = floor(v + 0.5)
            end
        else
            for i = 1, actuator_num do
                local v = output_buffer[i] * m
                if v > max_output then v = max_output
                elseif v < min_output then v = min_output end
                output_buffer[i] = v
            end
        end
    end
    parallel.waitForAll(table.unpack(self.actuators))
end

function controlAllocator:setOutputRange(min, max)
    if min > 0 or max < 0 then error("<controlAllocator> : Illegal Output Range") end
    self.min_output = min
    self.max_output = max
    --print(self.max_output)
end

return controlAllocator