-- camera-GPS.lua: Send GNSS coordinates to camera through custom serial protocol and control camera
-- Set CAM1_TYPE = 4 (Mount) to enable the camera driver

-- user selectable parameters
local DEBUG = 2
local BAUD_RATE = 115200
local INIT_INTERVAL_MS = 3000 -- attempt to initialise the camera and GPS at this interval
local UPDATE_INTERVAL_MS = 500 -- update at 2 Hz

-- global definitions
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}
local GPS_INSTANCE = gps:primary_sensor()
local MOUNT_INSTANCE = 0                -- always control the first mount/gimbal

-- packet parsing definitions
local HEADER_SEND = 0xAA -- header send
local HEADER_RECEIVE = 0x55 -- header receive

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER = 0
local PARSE_STATE_WAITING_FOR_OPERATION = 1
local PARSE_STATE_WAITING_FOR_LENGTH = 2
local PARSE_STATE_WAITING_FOR_DATA = 3

-- Camera Operation Bytes
local CAM_CMD_EXIF = 0x08
local CAM_PRM_EXIF_GPS = 0x01
local CAM_CMD_SHORTCUT = 0x01
local CAM_PRM_TAKE_PIC = 0x04
local CAM_PRM_REC_VIDEO = 0x05
local CAM_PRM_ZOOM = 0x08
local CAM_PRM_ZOOM_IN = 0x02
local CAM_PRM_ZOOM_OUT = 0x03

-- local variables and definitions
local uart  -- uart object connected to mount
local initialised = false -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER -- parse state
local parse_length = 0 -- incoming message parsed length
local parse_operation = 0 -- incoming message command id
local parse_data_buff = {} -- parsed data buffer
local parse_data_bytes_recv = 0 -- count of the number of bytes received in the message so far
local cam_pic_count = 0                 -- last picture count.  used to detect trigger pic
local cam_rec_video = false             -- last record video state.  used to detect record video
local cam_zoom_step = 0                 -- last zoom step state.  zoom out = -1, hold = 0, zoom in = 1
local cam_focus_step = 0                -- last focus step state.  focus in = -1, focus hold = 0, focus out = 1
local cam_autofocus = false             -- last auto focus state

-- parsing status reporting variables
local last_ms = 0 -- system time that debug output was last printed
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
local LEAP_SECONDS = 18 -- GPS leap seconds (leap seconds from 5/01/1980)
local GPS_TO_UNIX = 315964800
local UNIX_TO_2023 = 1672531200

function getLocalDateTime(time_week, time_week_ms)
  local time_s = math.floor((time_week_ms:toint()) / 1000)

  -- Convert GPS time to epoch and add leap seconds
  local epoch = (time_week * SECONDS_IN_WEEK + time_s) - LEAP_SECONDS + GPS_TO_UNIX - UNIX_TO_2023

  -- Calculate the number of days and seconds since the Unix epoch
  local days = math.floor(epoch / SECONDS_IN_DAY)
  local seconds = epoch % SECONDS_IN_DAY

  -- Calculate the number of years and days
  local years = 2023
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
  local second = seconds % SECONDS_IN_MINUTE

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

function checksum(packet, len)
  local t = 1
  local crc = 0x00

  while len > 0 do
    crc = crc ~ packet[t]
    t = t + 1
    for i = 8, 1, -1 do
      local msb = crc & 0x80
      crc = crc << 1

      if msb > 0 then
        crc = crc ~ 0xD5
      end
    end
    len = len - 1
  end

  return crc & 0xFF
end

-- send EXIF GPS message
function send_GPS()
  -- check GPS fix
  if gps:status(GPS_INSTANCE) < 3 and DEBUG < 3 then
    -- wait for GPS fix
    return false
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
  if DEBUG > 2 then
    lng = 1139359819
    lat = 225467631
    alt_sign = "+"
    alt = 100
    time = 111520
    gps_status = "V"
    velocity = 1234
    date = 20200511
  end

  if DEBUG > 1 then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("Long: %d, Lat: %d, Alt: %d, Vel: %d", lng, lat, alt, velocity))
  end

  -- define packet
  local packet_to_send = {
    HEADER_SEND, -- header
    31, -- length (including header and parity bit)
    CAM_CMD_EXIF, -- cmd 1
    CAM_PRM_EXIF_GPS, -- cmd 2
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
    0
  }

  -- calculate checksum
  packet_to_send[31] = checksum(packet_to_send, 30)

  -- send packet
  write_bytes(packet_to_send, #packet_to_send)
  return true
end

-- reading incoming packets from gimbal
function read_incoming_packets()
  local n_bytes = uart:available()
  while n_bytes > 0 do
    n_bytes = n_bytes - 1
    parse_byte(uart:read())
  end
end

-- check for changes in camera state and send messages if required
function check_camera_state()

  -- get latest state from AP driver
  local pic_count, rec_video, zoom_step, focus_step, auto_focus = mount:get_camera_state(MOUNT_INSTANCE)

  -- check for take picture
  if pic_count and pic_count ~= cam_pic_count then
    cam_pic_count = pic_count
    local packet_to_send = {HEADER_SEND, 0x05, CAM_CMD_SHORTCUT, CAM_PRM_TAKE_PIC, 0x0A}
    write_bytes(packet_to_send, #packet_to_send)
    if DEBUG > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("CAM: took pic %u", pic_count))
    end
  end

  -- check for start/stop recording video
  if rec_video ~= cam_rec_video then
    cam_rec_video = rec_video
    local packet_to_send = {HEADER_SEND, 0x05, CAM_CMD_SHORTCUT, CAM_PRM_REC_VIDEO, 0xDF}
    write_bytes(packet_to_send, #packet_to_send)
    if DEBUG > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "CAM: rec video:" .. tostring(cam_rec_video))
    end
  end
    
  -- check manual zoom
  -- zoom out = -1, hold = 0, zoom in = 1
  if zoom_step ~= cam_zoom_step then
    cam_zoom_step = zoom_step
    local packet_to_send = {HEADER_SEND, 0x06, CAM_CMD_SHORTCUT, CAM_PRM_ZOOM, 0, 0}
    if cam_zoom_step > 0 then
      packet_to_send[5] = CAM_PRM_ZOOM_IN
      packet_to_send[6] = 0x11
      write_bytes(packet_to_send, #packet_to_send)
    elseif cam_zoom_step < 0 then
      packet_to_send[5] = CAM_PRM_ZOOM_OUT
      packet_to_send[6] = 0xC4
      write_bytes(packet_to_send, #packet_to_send)
    end
    if DEBUG > 0 then
      gcs:send_text(MAV_SEVERITY.INFO, "CAM: zoom:" .. tostring(cam_zoom_step))
    end
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
          "CAM: %x %x %x %x %x %x %x %x %x %x",
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

  if DEBUG > 1 then
    gcs:send_text(MAV_SEVERITY.INFO, packet_string)
  end

  return true
end

-- the main update function
function update()
  -- initialise connection to gimbal
  if not initialised then
    init()
    return update, INIT_INTERVAL_MS
  end

  -- consume incoming bytes
  read_incoming_packets()

  -- camera control
  check_camera_state()

  -- send EXIF GPS
  if not send_GPS() then
    if DEBUG > 0 then
      gcs:send_text(MAV_SEVERITY.ERROR, "CAM: GPS not healthy")
    end
    return update, INIT_INTERVAL_MS
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
