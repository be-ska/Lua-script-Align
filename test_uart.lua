-- Driver for Custom Serial Rangefinder (NRA12)

-- User settable parameters

local INIT_MILLIS = 10
local UART_BAUD = 115200
local OUT_OF_RANGE_LOW = 0
local OUT_OF_RANGE_HIGH = 30

-- Global variables (DO NOT CHANGE)
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local UPDATE_MILLIS = 1000 -- update rate (in ms) of the driver
local lua_rfnd_driver_found = false

-- -------------------------------- RFND DRIVER --------------------------------

function init()
    -- find and init first instance of SERIALx_PROTOCOL = 28 (Scripting)
    uart = serial:find_serial(0)
    if uart == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "RFND: no SERIALx_PROTOCOL = 28")
        return
    else
        uart:begin(UART_BAUD)
        uart:set_flow_control(1)
        lua_rfnd_driver_found = true
    end
end

function read_incoming_bytes()
    local n_bytes = uart:available()
    local bytes_received = n_bytes
    local buffer = ""
    while n_bytes > 0 do
        local char = string.char(uart:read())
        buffer = buffer .. char
        n_bytes = n_bytes - 1
    end
    if bytes_received > 0 then
        gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("%d bytes received: " .. buffer, bytes_received:toint()))
    end

end

-- -------------------------------- MAIN --------------------------------

function update()
    if not lua_rfnd_driver_found then
        init()
        return update, INIT_MILLIS
    end

    -- consume incoming bytes
    read_incoming_bytes()

    return update, UPDATE_MILLIS
end

-- start running update loop
return update()
