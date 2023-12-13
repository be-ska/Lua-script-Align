-- control nozzle based on pump, check sprayer level

-- user parameters
local MILLIS_UPDATE = 1000
local LEVEL_PIN = 59 -- AP3 ch 10

-- global variables and init
gpio:pinMode(LEVEL_PIN,0) -- set level pin as input

function check_level()
    -- TODO: need to add data filtering
    -- set level state now
    local level_state_now = 0
    if gpio:read(LEVEL_PIN) then
        level_state_now = 1
    end
    if level_state_now == 1 then
        gcs:send_named_float("level", 1)
    else
        gcs:send_named_float("level", 0)
    end

    return check_level, MILLIS_UPDATE
end

return check_level, 1000
