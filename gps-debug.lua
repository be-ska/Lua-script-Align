-- control AP3/6 LEDs to debug the GPS status - version 1.0

local num_leds = 2
local chan = 12
local GPS_HDOP_GOOD = Parameter("GPS_HDOP_GOOD")
local AHRS_GPS_MINSATS = Parameter("AHRS_GPS_MINSATS")

serialLED:set_num_neopixel(chan, num_leds)

function check_gps()
    -- stop script if a valid RC input is found. has_valid_input() return true 
    -- if an RC input is not detected, hence the get_pwm(1) check
    if rc:has_valid_input() and rc:get_pwm(1) > 100 then
        return
    end

    -- check parameters for HDOP and SATS limits
    local hdop_good = GPS_HDOP_GOOD:get()
    local minsats = AHRS_GPS_MINSATS:get()
    if hdop_good == nil then
        hdop_good = 140
    end
    if minsats == nil then
        minsats = 6
    end

    if gps:status(gps:primary_sensor()) >= gps.GPS_OK_FIX_3D and gps:get_hdop(gps:primary_sensor()) <= hdop_good and gps:num_sats(gps:primary_sensor()) >= minsats then
        -- green
        serialLED:set_RGB(chan, 0, 0, 255, 0)
        serialLED:set_RGB(chan, 1, 0, 255, 0)
    elseif gps:status(gps:primary_sensor()) >= gps.GPS_OK_FIX_3D then
        -- blue
        serialLED:set_RGB(chan, 0, 0, 0, 255)
        serialLED:set_RGB(chan, 1, 0, 0, 255)
    elseif gps:status(gps:primary_sensor()) == gps.GPS_OK_FIX_2D then
        -- yellow
        serialLED:set_RGB(chan, 0, 255, 255, 0)
        serialLED:set_RGB(chan, 1, 255, 255, 0)
    elseif gps:status(gps:primary_sensor()) == gps.NO_FIX then
        -- orange
        serialLED:set_RGB(chan, 0, 255, 125, 0)
        serialLED:set_RGB(chan, 1, 255, 125, 0)
    else
        -- red
        serialLED:set_RGB(chan, 0, 255, 0, 0)
        serialLED:set_RGB(chan, 1, 255, 0, 0)
    end

    serialLED:send(chan)

    return check_gps, 1 -- run at 1000 Hz (need to override default LED code) 
end

gcs:send_text('6', "gps-debug.lua is running")
return check_gps, 500
