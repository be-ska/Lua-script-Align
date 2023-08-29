-- example script for using "get_origin()"
-- prints the home and ekf origin lat long and altitude to the console every 5 seconds

-- Constants for time calculations
local SECONDS_IN_MINUTE = 60
local SECONDS_IN_HOUR = 3600
local SECONDS_IN_DAY = 86400
local SECONDS_IN_WEEK = 604800
local LEAP_SECONDS = 19 -- GPS leap seconds (leap seconds from 5/01/1980)

-- Function to convert epoch timestamp to UTC date and time
function epochToUTC(epoch)
    -- Return the UTC date and time components
    return {
        year = year,
        month = month,
        day = day,
        hour = hour,
        minute = minute,
        second = second
    }
end

function getLocalDateTime(time_week, time_week_ms)

    local time_ms = time_week_ms:toint()
    local time_s = math.floor(time_ms/1000)

    -- Convert GPS time to epoch and add leap seconds
    local epoch = (time_week * SECONDS_IN_WEEK + time_s) + LEAP_SECONDS + 315964800

    -- Calculate the number of days and seconds since the Unix epoch
    local days = math.floor(epoch / SECONDS_IN_DAY)
    local seconds = epoch % SECONDS_IN_DAY
    gcs:send_text(0, string.format("Epoch: %d, time_ms: %d, Seconds: %d", epoch, time_ms, seconds))



    -- Calculate the number of years and days
    local years = 1970
    local daysInYear = 365
    while days >= daysInYear do
        if (years % 4 == 0) then
            daysInYear = 366 -- Leap year
        else
            daysInYear = 365 -- Non-leap year
        end
        days = days - daysInYear
        years = years + 1
    end
    -- Calculate the number of months and days
    local months = 1
    local daysInMonth = {31, 28 + (daysInYear == 366 and 1 or 0), 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
    while days >= daysInMonth[months] do
        days = days - daysInMonth[months]
        months = months + 1
    end
    -- Calculate the UTC date and time components
    local year = years
    local month = months
    local day = days + 1 -- Add 1 because days are 0-based
    local hour = math.floor(seconds / SECONDS_IN_HOUR)
    seconds = seconds % SECONDS_IN_HOUR
    local minute = math.floor(seconds / SECONDS_IN_MINUTE)
    local second = seconds % SECONDS_IN_MINUTE

    gcs:send_text(
        0,
        string.format(
            "Year: %d, Month: %d, Day: %d, Hour: %d, Minute: %d Second: %d, Seconds: %d",
            year, month, day, hour, minute, second, seconds
        )
    )    

end

function update()
    home = gps:location(0)
    velocity = (gps:velocity(0):x() ^ 2 + gps:velocity(0):y() ^ 2) ^ 0.5
    getLocalDateTime(gps:time_week(0), gps:time_week_ms(0))
    if home then
        gcs:send_text(0, string.format("Home - Lat:%d Long:%d Alt:%d", home:lat(), home:lng(), home:alt()))
        gcs:send_text(
            0,
            string.format(
                "Time: %d Time_ms: %d x:%d y:%d z:%d xy:%d",
                gps:time_week(0),
                gps:time_week_ms(0):toint(),
                gps:velocity(0):x(),
                gps:velocity(0):y(),
                gps:velocity(0):z(),
                velocity
            )
        )
    --gcs:send_text(0, string.format("Date: %d Time: ", date, time))
    end

    return update, 5000
end

return update()
