-- config_op.lua
-- 配置文件的加载、反序列化、默认值递归合并
-- 适用于 CC:Tweaked 环境

local M = {}

local function mergeTable(target, default, read_only)
    if not target or not default then
        return true, target or default or {}
    end

    if target == default then return false, target end

    local missing = false
    for key, defaultVal in pairs(default) do
        local targetVal = target[key]
        local targetType = type(targetVal)
        local defaultType = type(defaultVal)
        -- 目标缺失或类型不匹配，用默认值覆盖
        if targetVal == nil or targetType ~= defaultType then
            if not read_only then target[key] = defaultVal end
            missing = true
        -- 都是表，递归
        elseif defaultType == "table" then
            missing = mergeTable(targetVal, defaultVal, read_only) or missing
            if missing and read_only then return true, target end
        end
    end

    return missing, target
end

-- 加载配置文件
function M.load(filepath)
    -- 文件存在性检查
    if not fs.exists(filepath) then
        return {}, "ERROR: no such file"
    end
    -- 打开文件
    local file = fs.open(filepath, "r")
    if not file then
        return {}, "ERROR: open file failed"
    end
    -- 读取并关闭
    local content = file.readAll()
    file.close()
    -- 反序列化
    local ok, result = pcall(textutils.unserialize, content)
    if ok and type(result) == "table" then
        return result, "SUCCESS"
    else
    -- 反序列化失败
        return {}, "ERROR: unsupported file"
    end
end

-- 用默认表补全目标表
function M.merge(target, default)
    local _, result = mergeTable(target, default, false)
    return result
end

function M.checkArgs(target, default)
    local missing = mergeTable(target, default, true)
    return not missing
end

-- 用于查找外设
function M.getPeripheral(periphType, name)
    if not periphType then return end
    if not name then return peripheral.find(periphType) end
    local target_peripheral = peripheral.wrap(name)
    if not target_peripheral or peripheral.getType(name) ~= periphType then
        return peripheral.find(periphType)
    else
        return target_peripheral
    end
end

return M