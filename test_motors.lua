-- simple script to test servo outputs
-- WARNING: remove propellers when testing servo outputs!

-- user parameters
local PWM_STOP = 1300 -- pwm to stop servo outputs

-- global variables
local pwm = 1000

function update ()
    pwm = pwm + 1
    SRV_Channels:set_output_pwm(94, pwm)

    if pwm == PWM_STOP then
        SRV_Channels:set_output_pwm(94, 1000)
    else
        return update, 200
    end
end

gcs:send_text('0', "Test Motors is running, propellers should be removed!")
gcs:send_text('0', "Set output you want to test to SERVOx_FUNCTION = 94")
SRV_Channels:set_output_pwm(94, pwm)
return update, 5000
