-- control the blade distance from the ground of Align mowers - version 1.2

-- user parameters
local RELAY_PIN_UP = 00 -- TODO
local RELAY_PIN_DOWN = 00 -- TODO
local MILLIS_FULL_UP = 12000
local STEP_MAX = 6
local PWM_UP = 2000
local PWM_NEUTRAL = 1500
local PWM_DOWN = 1000
local MILLIS_UPDATE = 100

-- parameters
local PARAM_TABLE_KEY = 40
assert(param:add_table(PARAM_TABLE_KEY, "BLADE_", 7), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "CURR", 0), "could not add param 2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "SET", 0), "could not add param 3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "STEP", 2000), "could not add param 4")
assert(param:add_param(PARAM_TABLE_KEY, 5, "REV", 0), "could not add param 5")
assert(param:add_param(PARAM_TABLE_KEY, 6, "CH", 2), "could not add param 6")
assert(param:add_param(PARAM_TABLE_KEY, 7, "RELAY", 1), "could not add param 7")

-- bind parameters to variables
local BLADE_DEBUG = Parameter()
local BLADE_CURRENT_HEIGHT = Parameter()
local BLADE_SET_HEIGHT = Parameter()
local BLADE_STEP = Parameter()
local BLADE_REVERSE = Parameter()
local BLADE_CH = Parameter()
local BLADE_RELAY = Parameter()
BLADE_DEBUG:init("BLADE_DEBUG")
BLADE_CURRENT_HEIGHT:init("BLADE_CURR")
BLADE_SET_HEIGHT:init("BLADE_SET")
BLADE_STEP:init("BLADE_STEP")
BLADE_REVERSE:init("BLADE_REV")
BLADE_CH:init("BLADE_CH")
BLADE_RELAY:init("BLADE_RELAY")
local USE_RELAY = BLADE_RELAY:get()

-- global variables
local pwm_up = PWM_UP
local pwm_down = PWM_DOWN
local channel = 1

function set_output(pwm)
    if USE_RELAY > 0 then
        if pwm > PWM_NEUTRAL then
            gpio:write(RELAY_PIN_DOWN, 0)
            gpio:write(RELAY_PIN_UP, 1)
        elseif pwm < PWM_NEUTRAL then
            gpio:write(RELAY_PIN_UP, 0)
            gpio:write(RELAY_PIN_DOWN, 1)
        else
            gpio:write(RELAY_PIN_UP, 0)
            gpio:write(RELAY_PIN_DOWN, 0)
        end
    else
        SRV_Channels:set_output_pwm_chan(channel, pwm)
    end
end

function update()
    set_output(PWM_NEUTRAL)
    -- get params
    STEP_TO_MILLIS = BLADE_STEP:get()
    local current_height = BLADE_CURRENT_HEIGHT:get()
    local set_height = BLADE_SET_HEIGHT:get()

    -- check nil params
    if current_height == nil then
        BLADE_CURRENT_HEIGHT:set_and_save(0)
        if BLADE_DEBUG:get() > 0 then
            gcs:send_text('6', "BLADE_CURR is null")
        end
    end
    if set_height == nil then
        BLADE_SET_HEIGHT:set_and_save(0)
        if BLADE_DEBUG:get() > 0 then
            gcs:send_text('6', "BLADE_SET is null")
        end
    end

    -- check params limit
    if set_height < 0 then
        set_height = 0
    elseif set_height > STEP_MAX then
        set_height = STEP_MAX
    end

    if current_height < 1 or set_height < 1 then
        -- param out of range, go to homing procedure homing (move blade full up)
        if BLADE_DEBUG:get() > 0 then
            gcs:send_text('6', "Homing start")
        end
        BLADE_CURRENT_HEIGHT:set_and_save(STEP_MAX)
        BLADE_SET_HEIGHT:set_and_save(STEP_MAX)
        set_output(pwm_up)
        return update, MILLIS_FULL_UP
    elseif set_height > current_height then
        -- move blade up
        if BLADE_DEBUG:get() > 0 then
            gcs:send_text('6', "Move UP")
        end
        BLADE_CURRENT_HEIGHT:set_and_save(set_height)
        local timeout = (set_height-current_height)*STEP_TO_MILLIS
        if set_height == STEP_MAX then
            -- when going full up leave the servo for 500 ms longer, just to be sure to reach the maximum
            timeout = timeout + 500
        end
        set_output(pwm_up)
        return update, timeout
    elseif set_height < current_height then
        -- move blade down
        if BLADE_DEBUG:get() > 0 then
            gcs:send_text('6', "Move DOWN")
        end
        BLADE_CURRENT_HEIGHT:set_and_save(set_height)
        local timeout = (current_height-set_height)*STEP_TO_MILLIS
        if set_height == 1 then
            -- when going full down leave the servo for 500 ms longer, just to be sure to reach the minimum
            timeout = timeout + 500
        end
        set_output(pwm_down)
        return update, timeout
    end
    return update, MILLIS_UPDATE
end

gcs:send_text('6', "blade_distance.lua is running")

-- chek use relay
if USE_RELAY > 0 then
    gpio:pinMode(RELAY_PIN_UP,1) -- set RELAY pin as output
    gpio:pinMode(RELAY_PIN_DOWN,1) -- set RELAY pin as output
else
    -- use PWM output: check channel
    if BLADE_CH:get() == nil or BLADE_CH:get() < 1 then
        channel = 1
    else
    -- on old AP 4.2.x RC channels index is [0 15], newer firmwares is [1 16]
        channel = BLADE_CH:get() - 1
    end
end

-- be sure to start in neutral position
set_output(PWM_NEUTRAL)

-- check reverse parameter
if BLADE_REVERSE:get() > 0 then
    pwm_up = PWM_DOWN
    pwm_down = PWM_UP
end

return update, 1000