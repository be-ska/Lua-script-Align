-- This script simulate a rangefinder consuming the barometric altitude and adding an offset.
-- Used to test the surface_tracking reset

local INIT_MILLIS = 3000
local OFFSET = 2
-- Global variables (DO NOT CHANGE)
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local PARAM_LUA_RFND = 36 -- parameter number for lua rangefinder
local lua_rfnd_backend  -- store lua backend here
local lua_rfnd_driver_found = false -- true if user has configured lua backend
local UPDATE_MILLIS = 100 -- update rate (in ms) of the driver

-- -------------------------------- RFND DRIVER --------------------------------

function init()
    local sensor_count = rangefinder:num_sensors() -- number of sensors connected
    for j = 0, sensor_count - 1 do
        local device = rangefinder:get_backend(j)
        if ((not lua_rfnd_driver_found) and device and (device:type() == PARAM_LUA_RFND)) then
            -- this is a lua driver
            lua_rfnd_driver_found = true
            lua_rfnd_backend = device
        end
    end
    if not lua_rfnd_driver_found then
        -- We can't use this script if user hasn't setup a lua backend
        gcs:send_text(0, string.format("RFND: Configure RNGFNDx_TYPE = " .. PARAM_LUA_RFND))
    else
        gcs:send_text(MAV_SEVERITY.INFO, "RFND: started")
    end
end

function send_distance()
    dist = baro:get_altitude() + OFFSET
    local sent_successfully = lua_rfnd_backend:handle_script_msg(dist)
    if not sent_successfully then
        -- This should never happen as we already checked for a valid configured lua backend above
        gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("RFND: Lua Script Error"))
    end
    return send_distance, UPDATE_MILLIS
end

-- -------------------------------- MAIN --------------------------------

function update()
    if not lua_rfnd_driver_found then
        init()
        return update, INIT_MILLIS
    end
    return send_distance, UPDATE_MILLIS
end

-- start running update loop
return update()
