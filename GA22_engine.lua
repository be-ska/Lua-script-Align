-- emergency stop and fuel check for Align GA22

-- start engine -> relay low
-- stop engine -> relay high

-- user parameters
local MILLIS_UPDATE = 100
local START_PIN = 57 -- AP3 ch 8
local FUEL_PIN = 58 -- AP3 ch 9
local IGNITION_PIN = 59 -- AP3 ch 10
local IGNITION_ON = 1
local IGNITION_OFF = 0
local START_ON = 1
local START_OFF = 0

-- parameters
local PARAM_TABLE_KEY = 41
assert(param:add_table(PARAM_TABLE_KEY, "ENG_", 4), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "FUEL", 0), "could not add param 2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "IGN_RCIN", 8), "could not add param 3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "START_CH", 5), "could not add param 4")

-- bind parameters to variables
local ENG_DEBUG = Parameter()
local ENG_FUEL = Parameter()
local ENG_IGN_RCIN = Parameter()
local ENG_START_CH = Parameter()
ENG_DEBUG:init("ENG_DEBUG")
ENG_FUEL:init("ENG_FUEL")
ENG_IGN_RCIN:init("ENG_IGN_RCIN")
ENG_START_CH:init("ENG_START_CH")

-- global variables and init
local _ign_rcin = 0
local _start_ch = 0
local _fuel_state = -1
local _starter_first = true
gpio:pinMode(FUEL_PIN,0) -- set fuel pin as input
gpio:pinMode(IGNITION_PIN,1) -- set ignition pin as output
gpio:write(IGNITION_PIN, IGNITION_OFF) -- set ignition off
gpio:pinMode(START_PIN,1) -- set ignition pin as output
gpio:write(START_PIN, START_OFF) -- set starter pin off

function check_channels_params()
    _ign_rcin = ENG_IGN_RCIN:get()
    if _ign_rcin < 4 and _ign_rcin > 16 then
        gcs:send_text('3', "Select the RC channel for engine stop through param ENG_IGN_RCIN")
        return check_channels_params, 5000
    end

    _start_ch = ENG_START_CH:get()
    if _start_ch < 4 and _start_ch > 16 then
        gcs:send_text('3', "Select the RC channel for engine starter through param ENG_START_CH")
        return check_channels_params, 5000
    end

    return update, MILLIS_UPDATE
end

function set_ignition ()
    if rc:get_pwm(_ign_rcin) > 1600 then
        gpio:write(IGNITION_PIN, IGNITION_ON)
        if ENG_DEBUG:get() > 1 then
            gcs:send_text('6', "Ignition ON")
        end
    else
        gpio:write(IGNITION_PIN, IGNITION_OFF)
        if ENG_DEBUG:get() > 1 then
            gcs:send_text('6', "Ignition OFF")
        end
    end
end

function set_starter()
    if _starter_first then
        gpio:write(START_PIN, START_OFF)
        if rc:get_pwm(_start_ch) < 1400 then
            _starter_first = false
        end
    elseif rc:get_pwm(_start_ch) > 1600 then
        gpio:write(START_PIN, START_ON)
        if ENG_DEBUG:get() > 1 then
            gcs:send_text('6', "Starter ON")
        end
    else
        gpio:write(START_PIN, START_OFF)
        if ENG_DEBUG:get() > 1 then
            gcs:send_text('6', "Starter OFF")
        end
    end
end

function check_fuel()
    -- TODO: need to add data filtering
    -- set fuel state now
    local fuel_state_now = 0
    if gpio:read(FUEL_PIN) then
        fuel_state_now = 1
    end
    -- compare with fuel state saved, if different update parameter
    if fuel_state_now ~= _fuel_state then
        if fuel_state_now == 1 then
            ENG_FUEL:set_and_save(1)
            _fuel_state = 1
            if ENG_DEBUG:get() > 0 then
                gcs:send_text('6', "Fuel state 1")
            end
        else
            ENG_FUEL:set_and_save(0)
            _fuel_state = 0
            if ENG_DEBUG:get() > 0 then
                gcs:send_text('6', "Fuel state 0")
            end
        end
    end
end

function update()
    set_ignition()
    set_starter()
    check_fuel()
    return update, MILLIS_UPDATE
end

return check_channels_params, 1000
