-- custom arming routine for Align helicopters - version 1.0

-- constants
local AUX_FUNCTION_MOTOR_INTERLOCK = 32
local AUX_HIGH = 2
local AUX_LOW = 0
local STATES = {    WAIT_INTERLOCK_LOW = 0,
                    WAIT_INTERLOCK_HIGH = 1,
                    WAIT_THROTTLE_LOW = 2,
                    WAIT_THROTTLE_LOW_TIMEOUT = 3,
                    ARM = 4,
                    ARMED_ON = 5,
                    ARMED_OFF = 6 }

-- global variables
local state = STATES.WAIT_INTERLOCK_LOW
local time_ms = uint32_t(0)

function update()
    if state == STATES.WAIT_INTERLOCK_LOW then
        if rc:get_pwm(3) < 950 then
            -- rc not ready
            return update, 100
        end
        -- need motor interlock low after boot
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            state = STATES.WAIT_INTERLOCK_HIGH
        end

    elseif state == STATES.WAIT_INTERLOCK_HIGH then
        -- need motor interlock high before arming routine
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_HIGH then
            time_ms = millis()
            state = STATES.WAIT_THROTTLE_LOW
        end

    elseif state == STATES.WAIT_THROTTLE_LOW then
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            -- user turned off motor
            state = STATES.WAIT_INTERLOCK_HIGH
        end
        if rc:get_pwm(3) > 950 and rc:get_pwm(3) < 1150 then
            state = STATES.WAIT_THROTTLE_LOW_TIMEOUT
            time_ms = millis()
        end
        if millis() - time_ms > 10000 then
            -- timeout after 10 seconds
            state = STATES.WAIT_INTERLOCK_LOW
        end

    elseif state == STATES.WAIT_THROTTLE_LOW_TIMEOUT then
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            -- user turned off motor
            state = STATES.WAIT_INTERLOCK_HIGH
        end
        if rc:get_pwm(3) > 1150 then
            state = STATES.WAIT_THROTTLE_LOW
        end
        if millis() - time_ms > 2000 then
            rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_LOW)
            state = STATES.ARM
        end
 
    elseif state == STATES.ARM then
        if arming:arm() then
            -- motor interlock ON
            rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_HIGH)
            state = STATES.ARMED_ON
        else
            -- arm fail, go back to wait interlock low
            state = STATES.WAIT_INTERLOCK_LOW
        end

    elseif state == STATES.ARMED_ON then
        if not arming:is_armed() then
            state = STATES.WAIT_INTERLOCK_LOW
        end
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            -- user turned off motor
            time_ms = millis()
            state = STATES.ARMED_OFF
        end

    elseif state == STATES.ARMED_OFF then
        if not arming:is_armed() then
            state = STATES.WAIT_INTERLOCK_LOW
        end
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_HIGH then
            -- user turned on motor
            state = STATES.ARMED_ON
        end
        if millis() - time_ms > 10000 then
            -- try disarm after 10 seconds if vehicle is not flying
            -- if disarm fails wait 10 more seconds before trying again
            if not vehicle:get_likely_flying() then
                time_ms = millis()
                if arming:disarm() then
                    state = STATES.WAIT_INTERLOCK_LOW
                end
            end
        end
    end
    gcs:send_named_float("arm", state)
    return update, 100
end

gcs:send_text('6', "E1_arm.lua is running")
return update, 100