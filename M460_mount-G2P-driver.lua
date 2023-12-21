-- mount-G2P-driver.lua: Align G2P mount/gimbal driver

--[[
  How to use
    Connect gimbal UART to one of the autopilot's serial ports
    Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the gimbal
    Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
    Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
    Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
--]]

-- parameters
local PARAM_TABLE_KEY = 40
assert(param:add_table(PARAM_TABLE_KEY, "G2P_", 5), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DEBUG", 0), "could not add G2P_DEBUG param")

-- bind parameters to variables
local MNT1_TYPE = Parameter("MNT1_TYPE")    -- should be 9:Scripting
local G2P_DEBUG = Parameter("G2P_DEBUG")  -- debug level. 0:disabled 1:enabled 2:enabled with attitude reporting

-- MNT parametrs
local MNT1_PITCH_MAX = Parameter("MNT1_PITCH_MAX")
local MNT1_PITCH_MIN = Parameter("MNT1_PITCH_MIN")
local MNT1_ROLL_MAX = Parameter("MNT1_ROLL_MAX")
local MNT1_ROLL_MIN = Parameter("MNT1_ROLL_MIN")
local MNT1_YAW_MAX = Parameter("MNT1_YAW_MAX")
local MNT1_YAW_MIN = Parameter("MNT1_YAW_MIN")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local INIT_INTERVAL_MS = 3000           -- attempt to initialise the gimbal at this interval
local UPDATE_INTERVAL_MS = 20           -- update at 50hz
local MOUNT_INSTANCE = 0                -- always control the first mount/gimbal

-- packet parsing definitions
local HEADER_SEND = 0x18                    -- 1st header byte
local HEADER_RECEIVE = 0x19                 -- 2nd header byte
local PACKET_LENGTH_MIN = 6             -- serial packet minimum length.  used for sanity checks
local PACKET_LENGTH_MAX = 88            -- serial packet maximum length.  used for sanity checks

-- parsing state definitions
local PARSE_STATE_WAITING_FOR_HEADER    = 0
local PARSE_STATE_WAITING_FOR_OPERATION = 1
local PARSE_STATE_WAITING_FOR_LENGTH    = 2
local PARSE_STATE_WAITING_FOR_DATA      = 3

-- G2P Operation Bytes
local MOUNT_CMD_PARAM_SET      = 0x01
local MOUNT_CMD_PARAM_GET      = 0x02
local MOUNT_CMD_ANGLE_SET      = 0x03
local MOUNT_CMD_CALIBRATE      = 0x04


-- local variables and definitions
local uart                              -- uart object connected to mount
local initialised = false               -- true once connection to gimbal has been initialised
local parse_state = PARSE_STATE_WAITING_FOR_HEADER -- parse state
local parse_length = 0                  -- incoming message parsed length
local parse_data_bytes_recv = 0         -- count of the number of bytes received in the message so far
local roll = 0
local pitch = 0
local yaw = 0

-- parsing status reporting variables
local last_print_ms = 0                 -- system time that debug output was last printed
local bytes_read = 0                    -- number of bytes read from gimbal
local bytes_written = 0                 -- number of bytes written to gimbal
local bytes_error = 0                   -- number of bytes read that could not be parsed
local msg_ignored = 0                   -- number of ignored messages (because frame id does not match)

-- debug variables
local debug_count = 0                   -- system time that a test message was last sent
local debug_buff = {}                   -- debug buffer to display bytes from gimbal

-- get lowbyte of a number
function lowbyte(num)
  return num & 0xFF
end

-- get highbyte of a number
function highbyte(num)
  return (num >> 8) & 0xFF
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

-- wrap yaw angle in degrees to value between 0 and 360
function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

-- wrap yaw angle in degrees to value between -180 and +180
function wrap_180(angle_deg)
  local res = wrap_360(angle_deg)
  if res > 180 then
    res = res - 360
  end
  return res
end

-- calculate checksum
function calc_checksum(packet, len)
  local ck_a = 0;
  local ck_b = 0;
  for i = 2, len, 1 do
    ck_a = ck_a + packet[i];
    ck_b = ck_b + ck_a;
  end
  return ck_a, ck_b
end

-- find and initialise serial port connected to gimbal
function init()
  -- check mount parameter
  if MNT1_TYPE:get() ~= 9 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "G2P: set MNT1_TYPE=9")
    do return end
  end

  if MNT1_PITCH_MAX:get() == nil or MNT1_PITCH_MIN:get() == nil or MNT1_ROLL_MAX:get() == nil or MNT1_ROLL_MIN:get() == nil or MNT1_YAW_MAX:get() == nil or MNT1_YAW_MIN:get() == nil then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "G2P: check MNT1_ parameters")    
    do return end
  end

  -- find and init first instance of SERIALx_PROTOCOL = 28 (Scripting)
  uart = serial:find_serial(0)
  if uart == nil then
    gcs:send_text(3, "G2P: no SERIALx_PROTOCOL = 28") -- MAV_SEVERITY_ERR
  else
    uart:begin(115200)
    uart:set_flow_control(0)
    initialised = true
    gcs:send_text(MAV_SEVERITY.INFO, "G2P: started")
  end
end

-- send hard coded message
function send_msg(msg)
  for i=1,#msg do
    uart:write(msg[i])
    -- debug
    bytes_written = bytes_written + 1
  end
end

-- parse test message
function parse_test_msg(msg)
  for i=1,#msg do
    parse_byte(msg[i])
  end
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
    if G2P_DEBUG:get() > 2 then
      debug_buff[#debug_buff+1] = b
      if #debug_buff >= 10 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("G2P: %x %x %x %x %x %x %x %x %x %x", debug_buff[1], debug_buff[2], debug_buff[3], debug_buff[4], debug_buff[5], debug_buff[6], debug_buff[7], debug_buff[8], debug_buff[9], debug_buff[10]))
        debug_buff = {}
      end
    end

    -- waiting for header
    if parse_state == PARSE_STATE_WAITING_FOR_HEADER then
      if b == HEADER_RECEIVE then
        parse_state = PARSE_STATE_WAITING_FOR_OPERATION
        parse_data_bytes_recv = 1
        parse_data_buff[parse_data_bytes_recv] = b
        do return end
      end
      bytes_error = bytes_error + 1
    end

    -- waiting for operation
    if parse_state == PARSE_STATE_WAITING_FOR_OPERATION then
      if b == MOUNT_CMD_PARAM_SET or MOUNT_CMD_PARAM_GET or MOUNT_CMD_CALIBRATE then
        parse_state = PARSE_STATE_WAITING_FOR_LENGTH
        parse_operation = b
        parse_data_bytes_recv = parse_data_bytes_recv + 1
        parse_data_buff[parse_data_bytes_recv] = b
        
      else
        -- unexpected byte so reset parsing state
        parse_state = PARSE_STATE_WAITING_FOR_HEADER
        bytes_error = bytes_error + 1
      end
      do return end
    end

    -- waiting for length
    if parse_state == PARSE_STATE_WAITING_FOR_LENGTH then
      parse_length = b
      if parse_length >= PACKET_LENGTH_MIN and parse_length <= PACKET_LENGTH_MAX then
        parse_state = PARSE_STATE_WAITING_FOR_DATA
        parse_data_bytes_recv = parse_data_bytes_recv + 1
        parse_data_buff[parse_data_bytes_recv] = b
      else
        -- unexpected length
        parse_state = PARSE_STATE_WAITING_FOR_HEADER
        bytes_error = bytes_error + 1
        if G2P_DEBUG:get() > 0 then
          gcs:send_text(MAV_SEVERITY.ERROR, string.format("G2P: invalid len:%d", parse_length))
        end
      end
      do return end
    end

    -- waiting for data
    if parse_state == PARSE_STATE_WAITING_FOR_DATA then
      --TODO: receive payload bytes until length is 0
      parse_length = parse_length - 1
      parse_data_bytes_recv = parse_data_bytes_recv + 1
      parse_data_buff[parse_data_bytes_recv] = b
      if parse_length == 0 then
        --TODO: check checksum and do actions
        parse_state = PARSE_STATE_WAITING_FOR_HEADER
      end
    end
end

-- write a byte to the uart
function write_bytes(buff,len)
  if #buff == 0 or #buff < len then
    gcs:send_text(MAV_SEVERITY.ERROR, "G2P: failed to write byte") -- MAV_SEVERITY_ERR
    return false
  end

  local packet_string = "G2P packet send: "

  for i = 1, len, 1 do
    local byte_to_write = buff[i] & 0xFF
    uart:write(byte_to_write)
    bytes_written = bytes_written + 1
    packet_string = packet_string .. byte_to_write .. " "
  end

  if G2P_DEBUG:get() > 2 then
    gcs:send_text(MAV_SEVERITY.INFO, packet_string)
  end
  
  return true
end

-- send target angles (in degrees) to gimbal
-- yaw_angle_deg is always a body-frame angle
function send_target_angles(pitch_angle_deg, roll_angle_deg, yaw_angle_deg)
  if G2P_DEBUG:get() > 1 then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("G2P send angles P: %f R: %f Y; %f", pitch_angle_deg, roll_angle_deg, yaw_angle_deg))
  end

  -- convert angles from deg to G2P protocol
  local roll_angle_output = math.floor(roll_angle_deg * 80 + 32767 + 0.5)
  local pitch_angle_output = math.floor(pitch_angle_deg * 80 + 32767 + 0.5)
  local yaw_angle_output = math.floor(yaw_angle_deg * 80 + 32767 + 0.5)

  -- create packet
  local packet_to_send = {HEADER_SEND,
                          MOUNT_CMD_ANGLE_SET,
                          0x06,
                          highbyte(yaw_angle_output),
                          lowbyte(yaw_angle_output),
                          highbyte(roll_angle_output),
                          lowbyte(roll_angle_output),
                          highbyte(pitch_angle_output),
                          lowbyte(pitch_angle_output), 0, 0 }
  local ck_a, ck_b = calc_checksum(packet_to_send, 9)
  packet_to_send[10] = ck_a
  packet_to_send[11] = ck_b

  -- send packet
  write_bytes(packet_to_send, 11)
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

  -- send target angle to gimbal
  local roll_deg, pitch_deg, yaw_deg, yaw_is_ef = mount:get_angle_target(MOUNT_INSTANCE)
  local roll_degs, pitch_degs, yaw_degs, yaw_is_ef_rate = mount:get_rate_target(MOUNT_INSTANCE)
  if roll_deg and pitch_deg and yaw_deg then
    if yaw_is_ef then
      yaw_deg = wrap_180(yaw_deg - math.deg(ahrs:get_yaw()))
    end
    roll = roll_deg
    pitch = pitch_deg
    yaw = yaw_deg
    send_target_angles(pitch, roll, yaw)
  
  elseif roll_degs and pitch_degs and yaw_degs then
    roll = roll + roll_degs * UPDATE_INTERVAL_MS / 1000
    pitch = pitch + pitch_degs * UPDATE_INTERVAL_MS / 1000
    yaw = yaw + yaw_degs * UPDATE_INTERVAL_MS / 1000
    if roll > MNT1_ROLL_MAX:get() then
      roll = MNT1_ROLL_MAX:get()
    elseif roll < MNT1_ROLL_MIN:get() then
      roll = MNT1_ROLL_MIN:get()
    end
    if pitch > MNT1_PITCH_MAX:get() then
      pitch = MNT1_PITCH_MAX:get()
    elseif pitch < MNT1_PITCH_MIN:get() then
      pitch = MNT1_PITCH_MIN:get()
    end

    -- a sta merdata di gimbal per lo yaw come input diamo una velocità angolare, non si sa perché 
    yaw = yaw_degs * 3
    -- -- questo per quando lo sistemeranno
    -- if yaw > MNT1_YAW_MAX:get() then
    --   yaw = MNT1_YAW_MAX:get()
    -- elseif yaw < MNT1_YAW_MIN:get() then
    --   yaw = MNT1_YAW_MIN:get()
    -- end
    
    send_target_angles(pitch, roll, yaw)

  else
    gcs:send_text(MAV_SEVERITY.ERROR, "G2P: can't get target angles")
  end

  -- status reporting
  debug_count = debug_count + 1
  if (G2P_DEBUG:get() > 0) and (now_ms - last_print_ms > 5000) then
    last_print_ms = now_ms
    gcs:send_text(MAV_SEVERITY.INFO, string.format("G2P: r:%u w:%u err:%u ign:%u", bytes_read, bytes_written, bytes_error, msg_ignored))
    gcs:send_text(MAV_SEVERITY.INFO, string.format("G2P update frequency = %d Hz, last angle sent: R = %f, P = %f, Y = %f", debug_count//5, roll, pitch, yaw))
    debug_count = 0
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
