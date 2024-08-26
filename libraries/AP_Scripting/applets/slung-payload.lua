-- Lua script retrieve the position and velocity of a slung payload
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
-- 4. add an autopilot and GPS to the payload and configure it to send GLOBAL_POSITION_INT messages at 10hz to the vehicle
-- 5. create a mission with a SCRIPT_TIME or PAYLOAD_PLACE command included
-- 6. fly the mission and the vehicle should move so as to reduce the payload's oscillations while executing the SCRIPT_TIME or PAYLOAD_PLACE commands
-- 7. optionally set SLUP_SYSID to the system id of the payload autopilot
-- 8. optionally set WP_YAW_BEHAVIOR to 0 to prevent the vehicle from yawing while moving to the payload

-- load mavlink message definitions from modules/MAVLink directory
local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 100
local COPTER_MODE_AUTO = 3
local MAV_CMD_NAV_PAYLOAD_PLACE = 94
local MAV_CMD_NAV_SCRIPT_TIME = 42702
local PAYLOAD_OFFSET_COMP_POS_MAX = 50  -- payload offset compensation will be active when the payload is within this many meters of the vehicle
local PAYLOAD_OFFSET_COMP_VEL_MAX = 1   -- payload offset compensation will be active when the payload's horizontal velocity is no more than this speed in m/s

 -- setup script specific parameters
local PARAM_TABLE_KEY = 82
local PARAM_TABLE_PREFIX = "SLUP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
 end

--[[
  // @Param: SLUP_ENABLE
  // @DisplayName: Slung Payload enable
  // @Description: Slung Payload enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLUP_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: SLUP_POS_P
  // @DisplayName: Slung Payload Position P gain
  // @Description: Slung Payload Position P gain, higher values will result in faster movement towards the payload
  // @Range: 0 5
  // @User: Standard
--]]
local SLUP_POS_P = bind_add_param("POS_P", 2, 0.2)

--[[
  // @Param: SLUP_DIST_MAX
  // @DisplayName: Slung Payload horizontal distance max
  // @Description: Oscillation is suppressed when vehicle and payload are no more than this distance horizontally.  Set to 0 to always suppress
  // @Range: 0 30
  // @User: Standard
--]]
local SLUP_DIST_MAX = bind_add_param("DIST_MAX", 3, 15)

--[[
  // @Param: SLUP_SYSID
  // @DisplayName: Slung Payload mavlink system id
  // @Description: Slung Payload mavlink system id.  0 to use any/all system ids
  // @Range: 0 255
  // @User: Standard
--]]
local SLUP_SYSID = bind_add_param("SYSID", 4, 0)

--[[
  // @Param: SLUP_WP_POS_P
  // @DisplayName: Slung Payload return to WP position P gain
  // @Description: WP position P gain. higher values will result in vehicle moving more quickly back to the original waypoint.  Should always be lower than SLUP_POS_P
  // @Range: 0 1
  // @User: Standard
--]]
local SLUP_WP_POS_P = bind_add_param("WP_POS_P", 5, 0.05)

--[[
  // @Param: SLUP_RESTOFS_TC
  // @DisplayName: Slung Payload resting offset estimate filter time constant
  // @Description: payload's position estimator's time constant used to compensate for GPS errors and wind.  Higher values result in smoother estimate but slower response
  // @Range: 1 20
  // @User: Standard
--]]
local SLUP_RESTOFS_TC = bind_add_param("RESTOFS_TC", 6, 10)

--[[
  // @Param: SLUP_DEBUG
  // @DisplayName: Slung Payload debug output
  // @Description: Slung payload debug output, set to 1 to enable debug
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLUP_DEBUG = bind_add_param("DEBUG", 7, 0)

-- mavlink definitions
local HEARTBEAT_ID = 0
local GLOBAL_POSITION_INT_ID = 33
local msg_map = {}
msg_map[HEARTBEAT_ID] = "HEARTBEAT"
msg_map[GLOBAL_POSITION_INT_ID] = "GLOBAL_POSITION_INT"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(1, 10)

-- register message id to receive
mavlink:register_rx_msgid(HEARTBEAT_ID)
mavlink:register_rx_msgid(GLOBAL_POSITION_INT_ID)

-- variables
local resting_offset_NED = Vector3f()   -- estimated position offset
local resting_vel_NED = Vector3f()      -- estimated velocity offset.  should be near zero when hovering
local resting_offset_update_ms = uint32_t(0)    -- system time that resting_offset_NED was last updated
local resting_offset_notify_ms = uint32_t(0)    -- system time that the user was sent the resting_offset_NED
local found_heartbeat = false       -- true if a heartbeat message has been received
local found_payload_sysid = false   -- true if a global position int message has been received
local send_velocity_offsets = false -- true if we should send vehicle velocity offset commands to reduce payload oscillation
local send_velocity_offsets_prev = false -- previous value of send_velocity_offsets, used to detect changes and alert user
local wp_loc = nil      -- latest waypoint location

-- calculate an alpha for a first order low pass filter
function calc_lowpass_alpha(dt, time_constant)
    local rc = time_constant/(math.pi*2)
    return dt/(dt+rc)
 end

-- handle heartbeat message
function handle_heartbeat(msg)
    if not found_heartbeat then
        found_heartbeat = true
        gcs:send_text(MAV_SEVERITY.INFO, string.format("slung-payload: first heartbeat sysid:%d", msg.sysid))
    end
end

-- handle global position int message
function handle_global_position_int(msg)
    -- check if message is from the correct system id
    if (SLUP_SYSID:get() > 0 and msg.sysid ~= SLUP_SYSID:get()) then
        do return end
    end
    if not found_payload_sysid then
        found_payload_sysid = true
        gcs:send_text(MAV_SEVERITY.INFO, string.format("slung-payload: found sysid:%d", msg.sysid))
    end

    -- get payload location
    local payload_loc = Location()
    payload_loc:lat(msg.lat)
    payload_loc:lng(msg.lon)
    payload_loc:alt(msg.alt * 0.1)

    -- get payload velocity
    local payload_vel = Vector3f()
    payload_vel:x(msg.vx * 0.01)
    payload_vel:y(msg.vy * 0.01)
    payload_vel:z(msg.vz * 0.01)

    -- calculate position difference vs vehicle
    local curr_loc = ahrs:get_location()
    if curr_loc == nil then
        gcs:send_text(MAV_SEVERITY.WARNING, "slung-payload: failed to get vehicle location")
        do return end
    end
    local dist_NED = curr_loc:get_distance_NED(payload_loc)

    -- calculate payload's resting location
    update_payload_resting_offset(dist_NED, payload_vel)

    -- check horizontal distance is less than SLUP_DIST_MAX
    if SLUP_DIST_MAX:get() > 0 then
        local dist_xy = dist_NED:xy():length()
        if (dist_xy > SLUP_DIST_MAX:get()) then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("slung-payload: payload too far %4.1fm", dist_xy))
            do return end
        end
    end

    -- alert user if we start or stop sending velocity offsets
    if (send_velocity_offsets and not send_velocity_offsets_prev) then
        gcs:send_text(MAV_SEVERITY.INFO, "slung-payload: activated")
    end
    if (not send_velocity_offsets and send_velocity_offsets_prev) then
        gcs:send_text(MAV_SEVERITY.INFO, "slung-payload: stopped")
    end
    send_velocity_offsets_prev = send_velocity_offsets

    -- return early if not activated
    if not send_velocity_offsets then
        do return end
    end

    -- calculate distance to original waypoint.  used to slowly move back to waypoint
    local wp_dist_NED = Vector3f()
    if wp_loc then
        wp_dist_NED = curr_loc:get_distance_NED(wp_loc)
    end

    -- calculate payload offset used to compensate for GPS errors and wind
    local payload_offset_NED = Vector3f()
    if (millis() - resting_offset_update_ms < 1000) then
        -- check resting velocity is low enough to ensure resting position estimate is accurate
        if (resting_vel_NED:xy():length() <= PAYLOAD_OFFSET_COMP_VEL_MAX) then
            payload_offset_NED = resting_offset_NED
        elseif SLUP_DEBUG:get() > 0 then
            gcs:send_text(MAV_SEVERITY.WARNING, "slung-payload: resting velocity too high")
        end
    end
    if (payload_offset_NED:xy():length() > PAYLOAD_OFFSET_COMP_POS_MAX) then
        gcs:send_text(MAV_SEVERITY.WARNING, "slung-payload: payload offset too far, ignoring")
        payload_offset_NED:x(0)
        payload_offset_NED:y(0)
    end

    -- sanity check distance to WP (if too far this is likely a programming error)
    if (wp_dist_NED:xy():length() > 50) then
        gcs:send_text(MAV_SEVERITY.WARNING, "slung-payload: waypoint too far, ignoring")
        wp_dist_NED:x(0)
        wp_dist_NED:y(0)
        wp_loc = nil
    end

    -- send velocity offsets in m/s in NED frame
    local vel_offset_NED = Vector3f()
    vel_offset_NED:x((dist_NED:x() - payload_offset_NED:x()) * SLUP_POS_P:get() + (wp_dist_NED:x() - payload_offset_NED:x()) * SLUP_WP_POS_P:get())
    vel_offset_NED:y((dist_NED:y() - payload_offset_NED:y()) * SLUP_POS_P:get() + (wp_dist_NED:y() - payload_offset_NED:y()) * SLUP_WP_POS_P:get())
    if not vehicle:set_vel_offset(vel_offset_NED) then
        gcs:send_text(MAV_SEVERITY.ERROR, "slung-payload: failed to set vel offset")
    end
end

-- estimate the payload's resting position offset based on its current offset and velocity
function update_payload_resting_offset(pos_offset_NED, vel_NED)

    -- calculate dt since last update
    local now_ms = millis()
    local dt = (now_ms - resting_offset_update_ms):tofloat() * 0.001
    resting_offset_update_ms = now_ms

    -- sanity check dt
    if (dt <= 0) then
        do return end
    end

    -- if not updated for more than 1 second, reset resting offset to current offset
    if (dt > 1) then
        resting_offset_NED = pos_offset_NED
        resting_vel_NED = vel_NED
        do return end
    end

    -- use a low-pass filter to move the resting offset NED towards the pos_offset_NED
    local alpha = calc_lowpass_alpha(dt, SLUP_RESTOFS_TC:get())
    resting_offset_NED:x(resting_offset_NED:x() + (pos_offset_NED:x() - resting_offset_NED:x()) * alpha)
    resting_offset_NED:y(resting_offset_NED:y() + (pos_offset_NED:y() - resting_offset_NED:y()) * alpha)
    resting_offset_NED:z(resting_offset_NED:z() + (pos_offset_NED:z() - resting_offset_NED:z()) * alpha)
    resting_vel_NED:x(resting_vel_NED:x() + (vel_NED:x() - resting_vel_NED:x()) * alpha)
    resting_vel_NED:y(resting_vel_NED:y() + (vel_NED:y() - resting_vel_NED:y()) * alpha)
    resting_vel_NED:z(resting_vel_NED:z() + (vel_NED:z() - resting_vel_NED:z()) * alpha)

    -- update user
    if (now_ms - resting_offset_notify_ms > 3000) then
        resting_offset_notify_ms = now_ms
        if SLUP_DEBUG:get() > 0 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("resting a:%f px:%4.1f py:%4.1f pz:%4.1f vx:%4.1f vy:%4.1f vz:%4.1f", alpha, resting_offset_NED:x(), resting_offset_NED:y(), resting_offset_NED:z(), resting_vel_NED:x(), resting_vel_NED:y(), resting_vel_NED:z()))
        end
    end
end

-- display welcome message
gcs:send_text(MAV_SEVERITY.INFO, "slung-payload script loaded")

-- update function to receive location from payload and move vehicle to reduce payload's oscillation
function update()

    -- exit immediately if not enabled
    if (SLUP_ENABLE:get() <= 0) then
        return update, 1000
    end

    -- consume mavlink messages from payload
    local msg, _ = mavlink:receive_chan()
    if (msg ~= nil) then
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if (parsed_msg ~= nil) then
            if parsed_msg.msgid == HEARTBEAT_ID then
                handle_heartbeat(parsed_msg)
            end
            if parsed_msg.msgid == GLOBAL_POSITION_INT_ID then
                handle_global_position_int(parsed_msg)
            end
        end
    end

    -- check for nav script time command
    local armed_and_flying = arming:is_armed() and vehicle:get_likely_flying()
    local takingoff_or_landing = vehicle:is_landing() or vehicle:is_taking_off()
    local auto_mode = (vehicle:get_mode() == COPTER_MODE_AUTO)
    local scripting_or_payloadplace = (mission:get_current_nav_id() == MAV_CMD_NAV_SCRIPT_TIME) or (mission:get_current_nav_id() == MAV_CMD_NAV_PAYLOAD_PLACE)
    send_velocity_offsets = armed_and_flying and not takingoff_or_landing and auto_mode and scripting_or_payloadplace

    -- record waypoint location so vehicle can slowly return to it
    local target_loc = vehicle:get_target_location()
    if (target_loc ~= nil) then
        wp_loc = target_loc
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
