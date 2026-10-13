-- math_utils.lua
-- 一些常用操作
-- 适用于 CC:Tweaked 环境
local M = {}
local pi = math.pi
local two_pi = 2 * pi
local thr_pi = 3 * pi
local cos = math.cos
local sin = math.sin
local sqrt = math.sqrt

local function striCirClamp(rad)
    rad = (rad + pi) % two_pi
    return rad - pi
end

function M.clamp(value, min_value, max_value)
    if min_value > max_value then
        local k = min_value
        min_value = max_value
        max_value = k
    end
    if value > max_value then
        return max_value
    elseif value < min_value then
        return min_value
    else
        return value
    end
end

function M.circleClamp(rad)
    if rad > thr_pi or rad < - thr_pi then return striCirClamp(rad) end
    if rad > pi then return rad - 2 * pi
    elseif rad < -pi then return 2 * pi + rad
    else return rad end
end

function M.deadband(value, limit_v)
    if math.abs(value) < limit_v then
        return 0
    else
        return value
    end
end

function M.mix(a, b, alpha)
    if not (a and b and alpha) then return a end
    return a + alpha * (b - a)
end


-- 顺序为yaw，pitch，roll, ZYX约定
--- @param yaw number
--- @param pitch number
--- @param roll number
--- @param quaternion table
--- @return table
function M.eulerToQuat(yaw, pitch, roll, quaternion)
    if not quaternion then quaternion = {} end
    local half_yaw = yaw * 0.5
    local half_pit = pitch * 0.5
    local half_rol = roll * 0.5
    local cos_yaw = cos(half_yaw)
    local sin_yaw = sin(half_yaw)
    local cos_pit = cos(half_pit)
    local sin_pit = sin(half_pit)
    local cos_rol = cos(half_rol)
    local sin_rol = sin(half_rol)
    quaternion[1] = cos_rol * cos_pit * cos_yaw + sin_rol * sin_pit * sin_yaw
    quaternion[2] = sin_rol * cos_pit * cos_yaw - cos_rol * sin_pit * sin_yaw
    quaternion[3] = cos_rol * sin_pit * cos_yaw + sin_rol * cos_pit * sin_yaw
    quaternion[4] = cos_rol * cos_pit * sin_yaw - sin_rol * sin_pit * cos_yaw
    return quaternion
end

--- @param u_x number
--- @param u_y number
--- @param u_z number
--- @param theta number
--- @param quaternion table
--- @return table
function M.axisToQuat(u_x, u_y, u_z, theta, quaternion)
    if not quaternion then quaternion = {} end
    local half_theta = theta * 0.5
    local sin_half = sin(half_theta)
    local cos_half = cos(half_theta)
    quaternion[1] = cos_half
    quaternion[2] = u_x * sin_half
    quaternion[3] = u_y * sin_half
    quaternion[4] = u_z * sin_half
    return quaternion
end

--- @param after table
--- @param before table
--- @param result table
--- @return table
function M.quatMul(after, before, result)
    if not result then result = {} end
    local w_1, i_1, j_1, k_1 = after[1], after[2], after[3], after[4]
    local w_2, i_2, j_2, k_2 = before[1], before[2], before[3], before[4]
    result[1] = w_1 * w_2 - i_1 * i_2 - j_1 * j_2 - k_1 * k_2
    result[2] = w_1 * i_2 + i_1 * w_2 + j_1 * k_2 - k_1 * j_2
    result[3] = w_1 * j_2 - i_1 * k_2 + j_1 * w_2 + k_1 * i_2
    result[4] = w_1 * k_2 + i_1 * j_2 - j_1 * i_2 + k_1 * w_2
    return result
end

--- @param q table
--- @param v table
--- @param result table
--- @param reverse boolean
--- @return table
function M.quatRot(q, v, result, reverse)
    if not result then result = {} end
    local q_w, q_i, q_j, q_k, c_q_w, c_q_i, c_q_j, c_q_k
    local m_w, m_i, m_j, m_k
    local v_w, v_i, v_j, v_k = v[1], v[2], v[3], v[4]
    if reverse then q_w, q_i, q_j, q_k = q[1], -q[2], -q[3], -q[4]
    else q_w, q_i, q_j, q_k = q[1], q[2], q[3], q[4] end
    c_q_w, c_q_i, c_q_j, c_q_k = q_w, -q_i, -q_j, -q_k
    m_w = q_w * v_w - q_i * v_i - q_j * v_j - q_k * v_k
    m_i = q_w * v_i + q_i * v_w + q_j * v_k - q_k * v_j
    m_j = q_w * v_j - q_i * v_k + q_j * v_w + q_k * v_i
    m_k = q_w * v_k + q_i * v_j - q_j * v_i + q_k * v_w
    result[1] = m_w * c_q_w - m_i * c_q_i - m_j * c_q_j - m_k * c_q_k
    result[2] = m_w * c_q_i + m_i * c_q_w + m_j * c_q_k - m_k * c_q_j
    result[3] = m_w * c_q_j - m_i * c_q_k + m_j * c_q_w + m_k * c_q_i
    result[4] = m_w * c_q_k + m_i * c_q_j - m_j * c_q_i + m_k * c_q_w
    return result
end

--- @param q table
--- @param result table
--- @return table
function M.quatConj(q, result)
    if not result then result = {} end
    result[1] = q[1]
    result[2] = -q[2]
    result[3] = -q[3]
    result[4] = -q[4]
    return result
end

--- @param q table
--- @param result table
--- @return table
function M.quatNorm(q, result)
    if not result then result = {} end
    local one_over_length = 1 / sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4])
    result[1] = q[1] * one_over_length
    result[2] = q[2] * one_over_length
    result[3] = q[3] * one_over_length
    result[4] = q[4] * one_over_length
    return result
end

--- @param q table
--- @param result table
--- @return table
function M.quatNeg(q, result)
    if not result then result = {} end
    result[1] = -q[1]
    result[2] = -q[2]
    result[3] = -q[3]
    result[4] = -q[4]
    return result
end

--- @param q_1 table
--- @param q_2 table
--- @return number
function M.quatDot(q_1, q_2)
    return q_1[1] * q_2[1] + q_1[2] * q_2[2] + q_1[3] * q_2[3] + q_1[4] * q_2[4]
end

return M