-- logic to prevent a fence breach in all flight modes.
-- Parameters to set:
-- 1) FENCE_ENABLE = 1
-- 2) FENCE_ACTION = 0 (report only)

-- user parameters
local MILLIS_RTL = 2000 -- milliseconds after fence breach to RTL
local MILLIS_OUT = 1000 -- milliseconds after returning to the user flight modes

-- global variables
local state = FLYING
local last_millis
local fence_breach_first = true
local fence_breach_out_first = false
local breach_mode

-- flight modes
local LOITER_MODE = 5
local RTL_MODE = 6

-- states
FLYING = 0
FENCE_BREACH = 1
FENCE_RTL_INSIDE = 2
FENCE_RTL_OUTSIDE = 3

function flying()
    if fence:get_breaches() ~= 0 then
        fence_breach_first = true
        state = FENCE_BREACH
    end
end

function fence_breach()
    if fence_breach_first then
        last_millis = millis()
        breach_mode = vehicle:get_mode()
        fence_breach_first = false
    end
    if (millis() - last_millis > MILLIS_RTL) then
        state = FENCE_RTL_INSIDE
    end
end

function fence_rtl_inside()
    if vehicle:get_mode() ~= RTL_MODE then
        vehicle:set_mode(RTL_MODE)
    end
    if fence:get_breaches() == 0 then
        fence_breach_out_first = true
        state = FENCE_RTL_OUTSIDE
    end
end

function fence_rtl_outside()
    if fence_breach_out_first then
        last_millis = millis()
        fence_breach_out_first = false
    end
    if (millis() - last_millis > MILLIS_OUT) then
        state = FLYING
        vehicle:set_mode(breach_mode)
    end
end

function update()
    -- call the state functions
    if state == FLYING then
        flying()
    elseif state == FENCE_BREACH then
        fence_breach()
    elseif state == FENCE_RTL_INSIDE then
        fence_rtl_inside()
    elseif state == FENCE_RTL_OUTSIDE then
        fence_rtl_outside()
    end

    -- update at 5 Hz
    return update, 200
end

-- call the update function after startup
return update()