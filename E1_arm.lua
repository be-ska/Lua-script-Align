local AUX_FUNCTION_MOTOR_INTERLOCK = 32
local AUX_HIGH = 2
local AUX_LOW = 0
local state = 0
local time_ms = uint32_t(0)

local STATE_WAIT_INTERLOCK_LOW = 0
local STATE_WAIT_INTERLOCK_HIGH = 1
local STATE_WAIT_THROTTLE_LOW = 2
local STATE_WAIT_THROTTLE_LOW_TIMEOUT = 3
local STATE_ARM = 4
local STATE_ARMED = 5

function update()
    if state == STATE_WAIT_INTERLOCK_LOW then
        if rc:get_pwm(3) < 950 then
            -- rc not ready
            return update, 100
        end
        -- need motor interlock low after boot
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            state = STATE_WAIT_INTERLOCK_HIGH
        end

    elseif state == STATE_WAIT_INTERLOCK_HIGH then
        -- need motor interlock high before arming routine
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_HIGH then
            state = STATE_WAIT_THROTTLE_LOW
        end

    elseif state == STATE_WAIT_THROTTLE_LOW then
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            -- user turned off motor
            state = STATE_WAIT_INTERLOCK_HIGH
        end
        if rc:get_pwm(3) > 950 and rc:get_pwm(3) < 1150 then
            state = STATE_WAIT_THROTTLE_LOW_TIMEOUT
            time_ms = millis()
        end

    elseif state == STATE_WAIT_THROTTLE_LOW_TIMEOUT then
        if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
            -- user turned off motor
            state = STATE_WAIT_INTERLOCK_HIGH
        end
        if rc:get_pwm(3) > 1150 then
            state = STATE_WAIT_THROTTLE_LOW
        end
        if millis() - time_ms > 2000 then
            rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_LOW)
            state = STATE_ARM
        end
 
    elseif state == STATE_ARM then
        if arming:arm() then
            -- motor interlock ON
            rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_HIGH)
            state = STATE_ARMED
        else
            -- arm fail, go back to wait interlock low
            state = STATE_WAIT_INTERLOCK_LOW
        end

    elseif state == STATE_ARMED then
        if not arming:is_armed() then
            -- disarmed:
            -- 1) user turned off motor and disarmed occured due to timeout
            -- 2) automatic land turned off the motor, user should set motor
            --    interlock to low before triggering motor ON routine
            state = STATE_WAIT_INTERLOCK_LOW
        end
    end
    gcs:send_named_float("arm", state)
    return update, 100
end

gcs:send_text('6', "E1_arm.lua is running")
return update, 100