-- Example showing how a position offset can be added to a Copter's auto mission plan
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and in auto mode and then
-- adds an offset to the position or velocity target
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
--

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local copter_auto_mode_num = 3

-- adjust whether position or velocity offsets are used
local test_position = true    -- set to false to test velocity offsets
local position_offset_x = 5
local position_offset_y = 5
local velocity_offset_x = 0.5
local velocity_offset_y = 0.5

-- welcome message to user
gcs:send_text(MAV_SEVERITY.INFO, "copter-auto-posoffset.lua loaded")

function update()

  -- must be armed, flying and in auto mode
  if (not arming:is_armed()) or (not vehicle:get_likely_flying()) or (vehicle:get_mode() ~= copter_auto_mode_num) then
    return update, 1000
  end

  if test_position then
    -- set the position offset in meters in NED frame
    local pos_offset_NED = Vector3f()
    pos_offset_NED:x(position_offset_x)
    pos_offset_NED:y(position_offset_y)
    if not vehicle:set_pos_offset(pos_offset_NED) then
      gcs:send_text(MAV_SEVERITY.ERROR, "copter-auto-posoffset: failed to set pos offset")
    end
  else
    -- test velocity offsets in m/s in NED frame
    local vel_offset_NED = Vector3f()
    vel_offset_NED:x(velocity_offset_x)
    vel_offset_NED:y(velocity_offset_y)
    if not vehicle:set_vel_offset(vel_offset_NED) then
      gcs:send_text(MAV_SEVERITY.ERROR, "copter-auto-posoffset: failed to set vel offset")
    end
  end

  -- update at 1hz
  return update, 1000
end

return update()
