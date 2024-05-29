-- emergency stop, starter and fuel check for Align mower - version 2.1-beta
-- for GA22 ENG_FUEL need to be set to 1, for GA45 and GA80 to 0

-- user parameters
local MILLIS_UPDATE = 100
local START_PIN = 57 -- AP3 ch 8
local FUEL_PIN = 58 -- AP3 ch 9
local IGNITION_PIN = 59 -- AP3 ch 10

-- parameters
local PARAM_TABLE_KEY = 41
assert(param:add_table(PARAM_TABLE_KEY, "ENG_", 11), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "FUEL", 0), "could not add param 2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "CH", 6), "could not replace param 3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "REV_IGN", 1), "could not replace param 4")
assert(param:add_param(PARAM_TABLE_KEY, 5, "REV_STR", 0), "could not replace param 5")
assert(param:add_param(PARAM_TABLE_KEY, 6, "VIBE_OFF", 0.5), "could not replace param 6")
assert(param:add_param(PARAM_TABLE_KEY, 7, "VIBE_ON", 2), "could not replace param 7") -- no use
assert(param:add_param(PARAM_TABLE_KEY, 8, "THR_RC", 11), "could not replace param 8")
assert(param:add_param(PARAM_TABLE_KEY, 9, "THR_SR", 7), "could not replace param 9")
assert(param:add_param(PARAM_TABLE_KEY, 10, "THR_MIN", 1000), "could not replace param 10")
assert(param:add_param(PARAM_TABLE_KEY, 11, "THR_MAX", 2000), "could not replace param 11")



-- bind parameters to variables
local ENG_DEBUG = Parameter()
local ENG_FUEL = Parameter()
local ENG_REV_IGN = Parameter()
local ENG_REV_STR = Parameter()
local ENG_CH = Parameter()
local ENG_VIBE_OFF = Parameter()
local ENG_THR_RC = Parameter()
local ENG_THR_SR = Parameter()
local ENG_THR_MIN = Parameter()
local ENG_THR_MAX = Parameter()

ENG_DEBUG:init("ENG_DEBUG")
ENG_FUEL:init("ENG_FUEL")
ENG_CH:init("ENG_CH")
ENG_REV_IGN:init("ENG_REV_IGN")
ENG_REV_STR:init("ENG_REV_STR")
ENG_VIBE_OFF:init("ENG_VIBE_OFF")
ENG_THR_RC:init("ENG_THR_RC")
ENG_THR_SR:init("ENG_THR_SR")
ENG_THR_MAX:init("ENG_THR_MAX")
ENG_THR_MIN:init("ENG_THR_MIN")

-- costants
local MODE_HOLD = 4
local STARTER_TIMEOUT_ON_MS = 5000 -- starter timeout to prevent starter failure
local STARTER_TIMEOUT_OFF_MS = 8000 -- waiting time to run starter after timeout stopped it
local STARTER_TIMEOUT_LONG_MS = 20000 -- waiting time to run starter after STARTER_TIMEOUT_LONG_COUNT consecutive times
local STARTER_TIMEOUT_LONG_COUNT = 3
local STARTER_OFF = 0
local STARTER_ON = 1
local STARTER_ON_TIMEOUT = 2
local STARTER_OFF_TIMEOUT = 7
local STARTER_OFF_VIBE = 3
local STARTER_ON_VIBE = 4
local STARTER_ON_TIMEOUT_LONG = 5
local STARTER_OFF_TIMEOUT_LONG = 6
local STARTER_ON_THR_LOW = 8
local STARTER_OFF_THR_LOW = 9
local STARTER_INIT = -1

-- global variables
local eng_ch = 0
local fuel_state = -1
local ignition_on = 1
local ignition_off = 0
local start_on = 1
local start_off = 0
local starter_count = 0
local starter_state = STARTER_INIT
local starter_on_ms = uint32_t(0)
local starter_off_ms = uint32_t(0)
local last_gcs_send = uint32_t(0)

-- init gpio
if ENG_FUEL:get() > 0 then
    gpio:pinMode(FUEL_PIN,0) -- set fuel pin as input
end
gpio:pinMode(IGNITION_PIN,1) -- set ignition pin as output
gpio:write(IGNITION_PIN, ignition_off) -- set ignition off
gpio:pinMode(START_PIN,1) -- set ignition pin as output
gpio:write(START_PIN, start_off) -- set starter pin off

function check_channels_params()
    eng_ch = ENG_CH:get()
    if eng_ch < 4 and eng_ch > 16 then
        gcs:send_text('3', "Select the RC channel for engine stop through param ENG_CH")
        return check_channels_params, 5000
    end

    return update, MILLIS_UPDATE
end

function set_ignition()
    -- turn off engine if in hold mode (failsafe)
    if vehicle:get_mode() == MODE_HOLD then
        gpio:write(IGNITION_PIN, ignition_off)
        if ENG_DEBUG:get() > 3 then
            gcs:send_text('6', "Mode HOLD: Ignition OFF")
        end
    elseif rc:get_pwm(eng_ch) > 1350 then
        gpio:write(IGNITION_PIN, ignition_on)
        if ENG_DEBUG:get() > 3 then
            gcs:send_text('6', "Ignition ON")
        end
    else
        gpio:write(IGNITION_PIN, ignition_off)
        if ENG_DEBUG:get() > 3 then
            gcs:send_text('6', "Ignition OFF")
        end
    end
end

function set_starter()
    local vibrations = ahrs:get_vibration()
    -- init state, to be sure that machine is not turned on with starter ON
    if starter_state == STARTER_INIT then
        gpio:write(START_PIN, start_off)
        if rc:get_pwm(eng_ch) < 1800 then
            starter_off_ms = millis()
            starter_state = STARTER_OFF
        end

    -- OFF state, check vibration to be sure starter is not turned on while engine is ON
    elseif starter_state == STARTER_OFF then
        if vibrations:length() < ENG_VIBE_OFF:get() then
            if rc:get_pwm(eng_ch) > 1850 then
                starter_on_ms = millis()
                -- try starter for 3 times, wait a longer timeout
                if starter_count == STARTER_TIMEOUT_LONG_COUNT then
                    starter_state = STARTER_ON_TIMEOUT_LONG
                    if ENG_DEBUG:get() > 1 then
                        gcs:send_text('6', "Starter TIMEOUT LONG")
                    end
                -- throttle must be near maximum
                elseif ENG_THR_RC:get() > 3 and rc:get_pwm(ENG_THR_RC:get()) < 1850 then
                    starter_state = STARTER_ON_THR_LOW
                    if ENG_DEBUG:get() > 1 then
                        gcs:send_text('6', "Starter ON THR LOW")
                    end
                -- starter ON
                else
                    gpio:write(START_PIN, start_on)
                    starter_state = STARTER_ON
                    starter_count = starter_count+1
                    if ENG_DEBUG:get() > 1 then
                        gcs:send_text('6', "Starter ON")
                    end
                end
            end
        else
            starter_state = STARTER_OFF_VIBE
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Vibrations too HIGH, prevent starter")
            end
        end

        -- reset starter_count after STARTER_TIMEOUT_LONG_MS
        if (millis() - starter_off_ms > STARTER_TIMEOUT_LONG_MS) and (starter_count > 0) then
            starter_count = 0
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "starter count reset to 0")
            end
        end

    -- starter is running, turn off after timeout or if increase of vibrations signal the engine has started
    elseif starter_state == STARTER_ON then
        if rc:get_pwm(eng_ch) < 1800 then
            gpio:write(START_PIN, start_off)
            starter_off_ms = millis()
            starter_state = STARTER_OFF
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF")
            end
        elseif millis() - starter_on_ms > STARTER_TIMEOUT_ON_MS then
            gpio:write(START_PIN, start_off)
            starter_state = STARTER_ON_TIMEOUT
            starter_on_ms = millis()
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter TIMEOUT")
            end
        end

    elseif starter_state == STARTER_OFF_VIBE then
        if vibrations:length() < ENG_VIBE_OFF:get() then
            starter_off_ms = millis()
            starter_state = STARTER_OFF
                if ENG_DEBUG:get() > 1 then
                    gcs:send_text('6', "Vibrations OK")
                end
            elseif rc:get_pwm(eng_ch) > 1850 then
            starter_state = STARTER_ON_VIBE
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter VIBRATION ON")
            end
        end

    elseif starter_state == STARTER_ON_VIBE then
        if rc:get_pwm(eng_ch) < 1800 then
            starter_off_ms = millis()
            starter_state = STARTER_OFF
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF")
            end
        end

    -- timeout after engine start, user need to switch off starter and wait STARTER_TIMEOUT_OFF_MS
    elseif starter_state == STARTER_ON_TIMEOUT then
        if (rc:get_pwm(eng_ch) < 1800) then
            starter_state = STARTER_OFF_TIMEOUT
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF TIMEOUT")
            end
        end

    elseif starter_state == STARTER_OFF_TIMEOUT then
        if rc:get_pwm(eng_ch) > 1850 then
            starter_state = STARTER_ON_TIMEOUT
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter ON TIMEOUT")
            end
        elseif (millis() - starter_on_ms > STARTER_TIMEOUT_OFF_MS) then
            starter_off_ms = millis()
            starter_state = STARTER_OFF
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF")
            end
        end

    -- long timeout, if user turned on starter 3 consecutive times
    elseif starter_state == STARTER_ON_TIMEOUT_LONG then
        if rc:get_pwm(eng_ch) < 1800 then
            starter_state = STARTER_OFF_TIMEOUT_LONG
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF TIMEOUT LONG")
            end
        end

    elseif starter_state == STARTER_OFF_TIMEOUT_LONG then
        if rc:get_pwm(eng_ch) > 1850 then
            starter_state = STARTER_ON_TIMEOUT_LONG
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter ON TIMEOUT LONG")
            end
        elseif millis() - starter_on_ms > STARTER_TIMEOUT_LONG_MS then
            starter_count = 0
            starter_off_ms = millis()
            starter_state = STARTER_OFF
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF")
            end
        end

    elseif starter_state == STARTER_ON_THR_LOW then
        if rc:get_pwm(eng_ch) < 1800 then
            starter_state = STARTER_OFF_THR_LOW
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF THR LOW")
            end
        end
    
    elseif starter_state == STARTER_OFF_THR_LOW then
        if rc:get_pwm(eng_ch) > 1850 then
            starter_state = STARTER_ON_THR_LOW
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter ON THR LOW")
            end
        elseif rc:get_pwm(ENG_THR_RC:get()) > 1870 then
            starter_off_ms = millis()
            starter_state = STARTER_OFF
            if ENG_DEBUG:get() > 1 then
                gcs:send_text('6', "Starter OFF")
            end
        end

    else
        gcs:send_text('4', "Unknown starter state")
    end
end

function check_fuel()
    -- TODO: need to add data filtering
    -- set fuel state now
    local fuel_state_now = 0
    if gpio:read(FUEL_PIN) then
        fuel_state_now = 1
    end
    -- compare with fuel state saved, if different update
    if fuel_state_now ~= fuel_state then
        if fuel_state_now == 1 then
            fuel_state = 1
            if ENG_DEBUG:get() > 0 then
                gcs:send_text('6', "Fuel state 1")
            end
        else
            fuel_state = 0
            if ENG_DEBUG:get() > 0 then
                gcs:send_text('6', "Fuel state 0")
            end
        end
    end
end

function set_throttle()
    -- check maximum and minimum for reverse
    local pwm_out_max = ENG_THR_MAX:get()
    local pwm_out_min = ENG_THR_MIN:get()
    local reversed = false
    if pwm_out_max < pwm_out_min then
        reversed = true
        pwm_out_max = ENG_THR_MIN:get()
        pwm_out_min = ENG_THR_MAX:get()
    end

    -- nomralize range to [0 1]
    local pwm_in = rc:get_pwm(ENG_THR_RC:get())
    local pwm_in_norm
    if reversed then
        pwm_in_norm = (2000 - pwm_in) / 1000
    else
        pwm_in_norm = (pwm_in - 1000) / 1000
    end
    if pwm_in_norm < 0 then
        pwm_in_norm = 0
    elseif pwm_in_norm > 1 then
        pwm_in_norm = 1
    end

    -- scale output throttle
    local pwm_out_scaled = pwm_in_norm * (pwm_out_max - pwm_out_min) + pwm_out_min

    SRV_Channels:set_output_pwm_chan(ENG_THR_SR:get()-1, pwm_out_scaled)
end

function update()
    set_ignition()
    set_starter()
    if ENG_FUEL:get() > 0 then
        check_fuel()
    end
    if ENG_THR_RC:get() > 3 and ENG_THR_SR:get() > 3 then
        set_throttle()
    end
    if millis() - last_gcs_send > 1000 then
        last_gcs_send = millis()
        gcs:send_named_float("starter", starter_state)
        if ENG_FUEL:get() > 0 then
            gcs:send_named_float("fuel", fuel_state)
        end
    end
    return update, MILLIS_UPDATE
end

gcs:send_text('6', "engine.lua is running")
-- check reverse parameters
if ENG_REV_IGN:get() > 0 then
    ignition_on = 0
    ignition_off = 1
end
if ENG_REV_STR:get() > 0 then
    start_on = 0
    start_off = 1
end

return check_channels_params, 1000
