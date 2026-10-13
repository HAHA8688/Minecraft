sleep(0.1)
print("Welcome!")

rednet.open("bottom")
local flight_path = "/ProgramFiles/FlightController/"
local flight_name = "flight_controller.lua"
local instrument_path = "/ProgramFiles/Instrument/"
local instrument_name = "instrument.lua"
local flight_heartbeat_protocol = "FlightHeartbeat"
local instrument_heartbeat_protocol = "InstrumentHeartbeat"
local flight_exit_protocol = "FlightExit"
local instrument_exit_protocol = "InstrumentExit"

local local_id = os.getComputerID()
local heartbeat_overtime = 3.0
local receive_overtime = 2.0
local restart_wait = 1.0

local last_flight_heartbeat = os.clock()
local last_instrument_heartbeat = os.clock()
local flight_id, instrument_id
local now


local function bgRun(name, path)
    shell.setDir(path)
    shell.run("bg "..name)
end

local function flightSup()
    local _, time_stamp, _ = rednet.receive(flight_heartbeat_protocol, receive_overtime)
    if time_stamp then
        last_flight_heartbeat = now
        if not flight_id then
            flight_id = time_stamp
        elseif flight_id ~= time_stamp then
            rednet.send(local_id, flight_id, flight_exit_protocol)
            flight_id = time_stamp
        end
    end
end

local function instrumentSup()
    local _, time_stamp, _ = rednet.receive(instrument_heartbeat_protocol, receive_overtime)
    if time_stamp then
        last_instrument_heartbeat = now
        if not instrument_id then
            instrument_id = time_stamp
        elseif instrument_id ~= time_stamp then
            rednet.send(local_id, instrument_id, instrument_exit_protocol)
            instrument_id = time_stamp
        end
    end
end

bgRun(flight_name, flight_path)
bgRun(instrument_name, instrument_path)

while true do
    sleep(0.1)
    now = os.clock()

    parallel.waitForAll(flightSup, instrumentSup)

    if now - last_flight_heartbeat > heartbeat_overtime then
        last_flight_heartbeat = now + restart_wait
        rednet.send(local_id, flight_id, flight_exit_protocol)
        bgRun(flight_name, flight_path)
    elseif now - last_instrument_heartbeat > heartbeat_overtime then
        last_instrument_heartbeat = now + restart_wait
        rednet.send(local_id, instrument_id, instrument_exit_protocol)
        bgRun(instrument_name, instrument_path)
    end
end