-- control nozzle based on pump, check sprayer level

-- user parameters
local MILLIS_UPDATE = 100
local LEVEL_PIN = 59 -- AP3 ch 10

-- parameters
local PARAM_TABLE_KEY = 42
assert(param:add_table(PARAM_TABLE_KEY, "AGR_", 2), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "LEVEL", 0), "could not add param 2")

-- bind parameters to variables
local AGR_DEBUG = Parameter()
local AGR_LEVEL = Parameter()
AGR_DEBUG:init("AGR_DEBUG")
AGR_LEVEL:init("AGR_LEVEL")

-- global variables and init
local _level_state = -1
gpio:pinMode(LEVEL_PIN,0) -- set level pin as input

function check_level()
    -- TODO: need to add data filtering
    -- set level state now
    local level_state_now = 0
    if gpio:read(LEVEL_PIN) then
        level_state_now = 1
    end
    -- compare with level state saved, if different update parameter
    if level_state_now ~= _level_state then
        if level_state_now == 1 then
            AGR_LEVEL:set_and_save(1)
            _level_state = 1
            if AGR_DEBUG:get() > 0 then
                gcs:send_text('6', "Level state 1")
            end
        else
            AGR_LEVEL:set_and_save(0)
            _level_state = 0
            if AGR_DEBUG:get() > 0 then
                gcs:send_text('6', "Level state 0")
            end
        end
    end

    return check_level, MILLIS_UPDATE
end

return check_level, 1000
