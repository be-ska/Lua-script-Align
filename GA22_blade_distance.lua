-- control the blade distance from the ground of Align GA22

-- user parameters
local SERVO_HEIGHT = 1
local MILLIS_FULL_UP = 12000
local MILLIS_FULL_DOWN = 12000
local STEP_MAX = 6
local PWM_UP = 2000
local PWM_NEUTRAL = 1500
local PWM_DOWN = 1000
local MILLIS_UPDATE = 100

-- parameters
local PARAM_TABLE_KEY = 40
assert(param:add_table(PARAM_TABLE_KEY, "BLADE_", 4), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "CURR", 0), "could not add param 2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "SET", 0), "could not add param 3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "STEP", 2000), "could not add param 3")

-- bind parameters to variables
local BLADE_DEBUG = Parameter()
local BLADE_CURRENT_HEIGHT = Parameter()
local BLADE_SET_HEIGHT = Parameter()
local BLADE_STEP = Parameter()
BLADE_DEBUG:init("BLADE_DEBUG")
BLADE_CURRENT_HEIGHT:init("BLADE_CURR")
BLADE_SET_HEIGHT:init("BLADE_SET")
BLADE_STEP:init("BLADE_STEP")
local STEP_TO_MILLIS = BLADE_STEP:get()

-- global variables
local millis_update = MILLIS_UPDATE

function set_distance(step)
    if BLADE_DEBUG:get() > 0 then
        gcs:send_text('6', "set distance call")
    end
    if step > 0 then
        millis_update = step*STEP_TO_MILLIS
        SRV_Channels:set_output_pwm_chan_timeout(SERVO_HEIGHT, PWM_UP, millis_update)
    elseif step < 0 then
        millis_update = -step*STEP_TO_MILLIS
        SRV_Channels:set_output_pwm_chan_timeout(SERVO_HEIGHT, PWM_DOWN, millis_update)
    end
end

function home()
    if BLADE_DEBUG:get() > 0 then
        gcs:send_text('6', "home call")
    end
    SRV_Channels:set_output_pwm_chan_timeout(SERVO_HEIGHT, PWM_UP, MILLIS_FULL_UP)
    millis_update = MILLIS_FULL_UP
end

function update()
    SRV_Channels:set_output_pwm_chan(SERVO_HEIGHT, PWM_NEUTRAL)
    STEP_TO_MILLIS = BLADE_STEP:get()
    local current_height = BLADE_CURRENT_HEIGHT:get()
    local set_height = BLADE_SET_HEIGHT:get()
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
    millis_update = MILLIS_UPDATE

    if set_height ~= current_height then
        if set_height >= STEP_MAX then
            BLADE_CURRENT_HEIGHT:set_and_save(set_height)
            set_distance(STEP_MAX)
        elseif set_height == 1 then
            BLADE_CURRENT_HEIGHT:set_and_save(set_height)
            set_distance(-STEP_MAX)
        elseif current_height < 1 then
            BLADE_CURRENT_HEIGHT:set_and_save(STEP_MAX)
            home()
        else
            BLADE_CURRENT_HEIGHT:set_and_save(set_height)
            set_distance(set_height-current_height)        
        end
    end
    return update, millis_update
end

gcs:send_text('6', "blade_distance.lua is running")
SRV_Channels:set_output_pwm_chan(SERVO_HEIGHT, PWM_NEUTRAL)
return update, 1000