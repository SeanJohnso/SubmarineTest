# One waypoint of a mission published by task_node on jit/task/mission.
#
# Unlike WaypointXY this can carry a depth. The only publisher is task_node,
# which generates the depth on board from its own parameters. A list received
# from another vessel never supplies one.

float64 x      # metres East in local_origin, absolute
float64 y      # metres North in local_origin, absolute
bool has_z     # false: hold the depth commanded by the previous waypoint
float64 z      # metres Up from the depth at mission start (negative = deeper)
