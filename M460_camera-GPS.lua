-- camera-GPS.lua: Send GNSS coordinates to camera through custom serial protocol

-- user selectable parameters
local DEBUG = 2
local BAUD_RATE = 115200

-- global definitions
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local INIT_INTERVAL_MS = 3000 -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 500 -- update at 2 Hz
local GPS_INSTANCE = gps:primary_sensor()

-- packet parsing definitions
local HEADER_SEND = 0xAA -- header send
local HEADER_RECEIVE = 0x55 -- header receive

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER = 0
local PARSE_STATE_WAITING_FOR_OPERATION = 1
local PARSE_STATE_WAITING_FOR_LENGTH = 2
local PARSE_STATE_WAITING_FOR_DATA = 3

-- Camera Operation Bytes
local CAM_SET_EXIF = 0x08
local CAM_SET_EXIF_GPS = 0x01

-- local variables and definitions
local uart  -- uart object connected to mount
local initialised = false -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER -- parse state
local parse_length = 0 -- incoming message parsed length
local parse_operation = 0 -- incoming message command id
local parse_data_buff = {} -- parsed data buffer
local parse_data_bytes_recv = 0 -- count of the number of bytes received in the message so far

-- parsing status reporting variables
local last_print_ms = 0 -- system time that debug output was last printed
local bytes_read = 0 -- number of bytes read from gimbal
local bytes_written = 0 -- number of bytes written to gimbal
local bytes_error = 0 -- number of bytes read that could not be parsed
local msg_ignored = 0 -- number of ignored messages (because frame id does not match)

-- debug variables
local debug_count = 0 -- system time that a test message was last sent
local debug_buff = {} -- debug buffer to display bytes from gimbal

-- Constants for time calculations
local SECONDS_IN_MINUTE = 60
local SECONDS_IN_HOUR = 3600
local SECONDS_IN_DAY = 86400
local SECONDS_IN_WEEK = 604800
local LEAP_SECONDS = 19 -- GPS leap seconds (leap seconds from 5/01/1980)

function getLocalDateTime(time_week, time_week_ms)
  local time_ms = time_week_ms:toint()
  local time_s = math.floor(time_ms / 1000)

  -- Convert GPS time to epoch and add leap seconds
  local epoch = (time_week * SECONDS_IN_WEEK + time_s) - LEAP_SECONDS + 315964800

  -- Calculate the number of days and seconds since the Unix epoch
  local days = math.floor(epoch / SECONDS_IN_DAY)
  local seconds = epoch % SECONDS_IN_DAY

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
  days = days + 1 -- Add 1 because days are 0-based
  local hour = math.floor(seconds / SECONDS_IN_HOUR)
  seconds = seconds % SECONDS_IN_HOUR
  local minute = math.floor(seconds / SECONDS_IN_MINUTE)
  local second = math.floor(seconds % SECONDS_IN_MINUTE)

  local date = years * 10000 + months * 100 + days
  local time = hour * 10000 + minute * 100 + second
  return date, time
end

-- get lowbyte of a number
function byte1(num)
  return num & 0xFF
end

-- get highbyte of a number
function byte2(num)
  return (num >> 8) & 0xFF
end

-- get lowbyte of a number
function byte3(num)
  return (num >> 16) & 0xFF
end

-- get highbyte of a number
function byte4(num)
  return (num >> 24) & 0xFF
end

-- get uint16 from two bytes
function uint16_value(hbyte, lbyte)
  return ((hbyte & 0xFF) << 8) | (lbyte & 0xFF)
end

-- get int16 from two bytes
function int16_value(hbyte, lbyte)
  local uret = uint16_value(hbyte, lbyte)
  if uret <= 0x8000 then
    return uret
  else
    return uret - 0x10000
  end
end

-- calculate checksum
function calc_checksum(packet, len)
  local ck_a = 0
  local ck_b = 0
  for i = 2, len, 1 do
    ck_a = ck_a + packet[i]
    ck_b = ck_b + ck_a
  end
  return ck_a, ck_b
end

-- find and initialise serial port connected to gimbal
function init()
  -- find and init second instance of SERIALx_PROTOCOL = 28 (Scripting)
  uart = serial:find_serial(1)
  if uart == nil then
    gcs:send_text(MAV_SEVERITY.ERROR, "CAM: need two SERIALx_PROTOCOL = 28") -- MAV_SEVERITY_ERR
  else
    uart:begin(BAUD_RATE)
    uart:set_flow_control(0)
    initialised = true
    gcs:send_text(MAV_SEVERITY.INFO, "CAM: started")
  end
end

-- send EXIF GPS message
function send_GPS()
  -- check GPS fix
  if gps:status(GPS_INSTANCE) < 3 and DEBUG < 1 then
    -- wait for GPS fix
    return
  end

  -- calculate packet variables
  local location = gps:location(GPS_INSTANCE)
  local lat = location:lat() -- deg E7
  local lng = location:lng() -- deg E7
  local alt = location:alt() // 100 -- m
  local alt_sign = "+"
  if alt < 0 then
    alt_sign = "-"
  end
  local gps_status = "V"
  local date, time = getLocalDateTime(gps:time_week(GPS_INSTANCE), gps:time_week_ms(GPS_INSTANCE))
  local velocity_vec = gps:velocity(0) -- velocity vector
  local velocity = math.floor(((velocity_vec:x() ^ 2 + velocity_vec:y() ^ 2) ^ 0.5) * 3600 + 0.5) -- km/h E3

  -- debug packet, override with default datas
  if DEBUG > 1 then
    lng = 1139359819
    lat = 225467631
    alt_sign = "+"
    alt = 100
    time = 111520
    gps_status = "V"
    velocity = 1234
    date = 20200511
  end

  -- define packet
  local packet_to_send = {
    HEADER_SEND, -- header
    31, -- length (including header and parity bit)
    CAM_SET_EXIF, -- cmd 1
    CAM_SET_EXIF_GPS, -- cmd 2
    byte1(lng),
    byte2(lng),
    byte3(lng),
    byte4(lng),
    byte1(lat),
    byte2(lat),
    byte3(lat),
    byte4(lat),
    string.byte(alt_sign),
    byte1(alt),
    byte2(alt),
    byte3(alt),
    byte4(alt),
    byte1(time),
    byte2(time),
    byte3(time),
    byte4(time),
    string.byte(gps_status),
    byte1(velocity),
    byte2(velocity),
    byte3(velocity),
    byte4(velocity),
    byte1(date),
    byte2(date),
    byte3(date),
    byte4(date),
    168
  }

  -- send packet
  write_bytes(packet_to_send, #packet_to_send)
end

-- reading incoming packets from gimbal
function read_incoming_packets()
  local n_bytes = uart:available()
  while n_bytes > 0 do
    n_bytes = n_bytes - 1
    parse_byte(uart:read())
  end
end

-- parse a single byte from gimbal
function parse_byte(b)
  -- record num bytes for reporting
  bytes_read = bytes_read + 1

  -- debug
  if DEBUG > 0 then
    debug_buff[#debug_buff + 1] = b
    if #debug_buff >= 10 then
      gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
          "G2P: %x %x %x %x %x %x %x %x %x %x",
          debug_buff[1],
          debug_buff[2],
          debug_buff[3],
          debug_buff[4],
          debug_buff[5],
          debug_buff[6],
          debug_buff[7],
          debug_buff[8],
          debug_buff[9],
          debug_buff[10]
        )
      )
      debug_buff = {}
    end
  end
end

-- write byte to uart
function write_bytes(buff, len)
  if #buff == 0 or #buff < len then
    gcs:send_text(MAV_SEVERITY.ERROR, "CAM: failed to write byte") -- MAV_SEVERITY_ERR
    return false
  end

  local packet_string = "CAM packet send: "

  for i = 1, len, 1 do
    local byte_to_write = buff[i]
    uart:write(byte_to_write)
    bytes_written = bytes_written + 1
    packet_string = packet_string .. byte_to_write .. " "
  end

  if DEBUG > 0 then
    gcs:send_text(MAV_SEVERITY.INFO, packet_string)
  end

  return true
end

-- the main update function
function update()
  -- get current system time
  local now_ms = millis()

  -- initialise connection to gimbal
  if not initialised then
    init()
    return update, INIT_INTERVAL_MS
  end

  -- consume incoming bytes
  read_incoming_packets()

  -- send EXIF GPS
  send_GPS()

  return update, UPDATE_INTERVAL_MS
end

return update()
