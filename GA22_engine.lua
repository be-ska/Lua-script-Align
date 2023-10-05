-- emergency stop and fuel check for Align GA22

-- start engine -> relay low
-- stop engine -> relay high

-- user parameters
local MILLIS_UPDATE = 100
local FUEL_PIN = 58 -- AP3 ch 9
local IGNITION_PIN = 59 -- AP3 ch 10

-- parameters
local PARAM_TABLE_KEY = 41
assert(param:add_table(PARAM_TABLE_KEY, "ENG_", 3), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "FUEL", 0), "could not add param 2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "IGN_RCIN", 8), "could not add param 3")

-- bind parameters to variables
local ENG_DEBUG = Parameter()
local ENG_FUEL = Parameter()
local ENG_IGN_RCIN = Parameter()
ENG_DEBUG:init("ENG_DEBUG")
ENG_FUEL:init("ENG_FUEL")
ENG_IGN_RCIN:init("ENG_IGN_RCIN")

-- global variables and init
local _ign_rcin = 0
local _fuel_state = -1
gpio:pinMode(FUEL_PIN,0) -- set fuel pin as input
gpio:pinMode(IGNITION_PIN,1) -- set ignition pin as output

function check_ign_rcin()
    _ign_rcin = ENG_IGN_RCIN:get()
    if _ign_rcin > 4 and _ign_rcin < 16 then
        return update, 100
    end
    gcs:send_text('3', "Select the RC channel for engine stop through param ENG_IGN_RCIN")
    return check_ign_rcin, 5000
end

function set_ignition ()
    if rc:get_pwm(_ign_rcin) > 1600 then
        gpio:write(IGNITION_PIN, 1)
        if ENG_DEBUG:get() > 1 then
            gcs:send_text('6', "Ignition ON")
        end
    else
        gpio:write(IGNITION_PIN, 0)
        if ENG_DEBUG:get() > 1 then
            gcs:send_text('6', "Ignition OFF")
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
    check_fuel()
    return update, 100
end

return check_ign_rcin, 1000
