# Slung Payload

This script reduces the oscillation of a slung payload that is capable of sending its position and velocity to the main vehicle

# Parameters

SLUP_ENABLE : Set to 1 to enable this script
SLUP_POS_P : Oscillation controller position P gain.  Higher values result in the vehicle moving more quickly towards the payload
SLUP_DIST_MAX : maximum acceptable distance between vehicle and payload.  Within this distance oscillation suppression will operate
SLUP_SYSID : System id of payload's autopilot.  If zero any system id is accepted
SLUP_WP_POS_P : Return to waypoint position P gain.  Higher values result in the vehicle returning more quickly to the latest waypoint
SLUP_RESTOFS_TC : Slung Payload resting offset estimate filter time constant.  Higher values result in smoother estimate but slower response
SLUP_DEBUG : Slung payload debug output, set to 1 to enable debug

# How To Use

1. mount an autopilot on the payload connected to the main vehicle using telemetry
2. ensure the vehicle and payload autopilots have unique system ids
3. copy this script to the vehicle autopilot's "scripts" directory
4. within the "scripts" directory create a "modules" directory
5. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory

# How It Works

The script's algorithm is implemented as follows

1. Consume GLOBAL_POSITION_INT messages from the payload
2. Calculate the payload's position vs the vehicle position
3. Use a P controller to move the vehicle towards the payload to reduce oscillation
4. Simultaneously the vehicle moves back towards the original location.  The speed depends upon the SLUP_WP_POS_P parameter
