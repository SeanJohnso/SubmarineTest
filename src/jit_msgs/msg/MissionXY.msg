# A horizontal-only mission, published by waypoint_listener_node on
# jit/nav/local/mission. header.frame_id is the sub's local frame
# (local_origin); every waypoint has already been transformed into it.
#
# No z anywhere, by design - see WaypointXY.msg.

std_msgs/Header header
WaypointXY[] waypoints

# Heading of the last waypoint in the list, as received from the other vessel
# and rotated into local_origin. Radians, ENU: 0 = East, counter-clockwise.
float64 final_yaw
