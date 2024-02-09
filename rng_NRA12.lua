-- Driver for Custom Serial Rangefinder (NRA12)

-- User settable parameters
local INIT_MILLIS = 3000
local UART_BAUD = 115200
local OUT_OF_RANGE_HIGH = 20

-- Constants
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local PARAM_LUA_RFND = 36 -- parameter number for lua rangefinder
local PARAM_TABLE_KEY = 72
local HEADER0 = 0xAA
local HEADER1 = 0xAA
local HEADER2_TARGET_INFO = 0x0C
local HEADER3_TARGET_INFO = 0x07
local END1 = 0x55
local END2 = 0x55

-- Parameters
assert(param:add_table(PARAM_TABLE_KEY, "NRA_", 3), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, "LOG", 0), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY, 3, "UPDATE", 24), 'could not add param3')
local NRA_DEBUG = Parameter("NRA_DEBUG")
local NRA_LOG = Parameter("NRA_LOG")
local NRA_UPDATE = Parameter("NRA_UPDATE")

-- Global variables
local lua_rfnd_backend  -- store lua backend here
local lua_rfnd_driver_found = false -- true if user has configured lua backend
local parse_state = 0
local distance = 0
local distance_received = false
local uart = nil
local header_found_ms = 0
local distance_received_ms = 0

---------------------------------- RFND DRIVER --------------------------------

function init_rng()
    local sensor_count = rangefinder:num_sensors() -- number of sensors connected
    for j = 0, sensor_count - 1 do
        local device = rangefinder:get_backend(j)
        if ((not lua_rfnd_driver_found) and device and (device:type() == PARAM_LUA_RFND)) then
            -- this is a lua driver
            lua_rfnd_driver_found = true
            lua_rfnd_backend = device
            gcs:send_text(MAV_SEVERITY.INFO, "RFND: backend found")
        end
    end

    if not lua_rfnd_driver_found then
        -- We can't use this script if user hasn't setup a lua backend
        gcs:send_text(0, string.format("RFND: Configure RNGFNDx_TYPE = " .. PARAM_LUA_RFND))
        return update, INIT_MILLIS
    else
        return update, NRA_UPDATE:get()
    end
end

function read_incoming_bytes()
    local now = millis()
    local n_bytes = uart:available()
    while n_bytes > 0 do
        n_bytes = n_bytes-1
        local byte = uart:read()
        if parse_state == 0 then
            if byte == HEADER0 then
                parse_state = 1
            end
        elseif parse_state == 1 then
            if byte == HEADER1 then
                parse_state = 2
                header_found_ms = now
            else
                parse_state = 0
            end
        elseif parse_state == 2 then
            if byte == HEADER2_TARGET_INFO then
                parse_state = 3
            else
                parse_state = 0
            end
        elseif parse_state == 3 then
            if byte == HEADER3_TARGET_INFO then
                parse_state = 4
            else
                parse_state = 0
            end
        -- skip index and RCS
        elseif parse_state < 6 then
            parse_state = parse_state + 1
        elseif parse_state == 6 then
            distance = byte * 256
            parse_state = 7
        elseif parse_state == 7 then
            distance = distance + byte
            parse_state = 8
         -- skip Rsvd1, roll count, Verl, SNR
        elseif parse_state < 12 then
            parse_state = parse_state + 1
        elseif parse_state == 12 then
            if byte == END1 then
                parse_state = 13
            else
                parse_state = 0
            end
        elseif parse_state == 13 then
            if byte == END2 then
                distance_received = true
                distance_received_ms = now
            end
            parse_state = 0
        end
    end
end

function send_distance(distance_m)
    if distance_m > 20 then
        distance_m = 20
    end
    sent_successfully = lua_rfnd_backend:handle_script_msg(distance_m)
    if not sent_successfully then
        -- This should never happen as we already checked for a valid configured lua backend above
        gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("RFND: Lua Script Error"))
    end
end

-- -------------------------------- MAIN --------------------------------

function update()
    if not lua_rfnd_driver_found then
        return init_rng, INIT_MILLIS
    end

    if uart == nil then
        uart = serial:find_serial(0)
        if uart == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, "RFND: configure SERIALx_PROTOCOL = 28")
            return update, INIT_MILLIS
        else
            uart:begin(UART_BAUD)
            uart:set_flow_control(0)
            -- first flush the serial buffer
            while uart:available()>0 do
                uart:read()
            end
            gcs:send_text(MAV_SEVERITY.INFO, "RFND: succesfully started")
        end
    end

    -- consume incoming bytes
    read_incoming_bytes()
    local now = millis()
    if distance_received then
        send_distance(distance/100)
        distance_received = false
        -- TODO: add logging if needed
        -- if NRA_LOG:get() > 0 then
        --     local dist_log = math.floor(distance)
        --     local mag_log = math.floor(magnitude)
        --     logger:write("RDR", "mag, dist", "hh", mag_log, dist_log)
        -- end
    elseif now - distance_received_ms > 100 and now - header_found_ms < 50 then
        send_distance(OUT_OF_RANGE_HIGH)
    end

    return update, NRA_UPDATE:get()
end

-- start running update loop
return update, INIT_MILLIS
