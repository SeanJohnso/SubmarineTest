# A mission from task_node to local_guided_node, on jit/task/mission.
# header.frame_id is the sub's local frame (local_origin).
#
# Deliberately not latched: local_guided_node starts this the moment it arrives,
# so a restarted node must not be handed an old one.

std_msgs/Header header
TaskWaypoint[] waypoints
