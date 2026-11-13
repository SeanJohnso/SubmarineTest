#!/usr/bin/env python3
"""Bridge the sub's private DDS domain to the shared one.

The sub (MAVROS and the jit stack) runs on ``private_domain``. The boat and the
other robots run on ``shared_domain``. ``config/domain_bridge.yaml`` lists the
topics that cross and in which direction.

The bridge's own two nodes are placed on the domains named here, whatever
ROS_DOMAIN_ID is. Every other process on the sub must be started with
ROS_DOMAIN_ID set to ``private_domain``, or it is on neither side of the bridge;
the launch warns if this shell's ROS_DOMAIN_ID does not match.

Started by jit_bringup.launch.py unless ``enable_domain_bridge:=false``. Run it
on its own when MAVROS is up without the rest of the stack:

    export ROS_DOMAIN_ID=77
    ros2 launch jit_bringup domain_bridge.launch.py
    ros2 launch jit_bringup domain_bridge.launch.py shared_domain:=56 private_domain:=77

Needs ``sudo apt install ros-humble-domain-bridge``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RESPAWN_DELAY_S = 2.0


def check_domain(context):
    """Warn when this shell is not on the private domain."""
    private = LaunchConfiguration("private_domain").perform(context)
    current = os.environ.get("ROS_DOMAIN_ID", "0") or "0"
    if current == private:
        return []
    return [LogInfo(
        msg="[domain_bridge] WARNING: ROS_DOMAIN_ID is {} but private_domain is {}. "
            "Nodes started from this shell are not on the sub's side of the bridge. "
            "Run 'export ROS_DOMAIN_ID={}' before launching MAVROS and the stack."
            .format(current, private, private)
    )]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "shared_domain", default_value="56",
            description="DDS domain of the boat and the other robots."),
        DeclareLaunchArgument(
            "private_domain", default_value="77",
            description="DDS domain of the sub: MAVROS and the jit stack."),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("jit_bringup"), "config", "domain_bridge.yaml"]),
            description="Topics to bridge and their direction."),
    ]

    # No name=: the process holds one node per domain, named from the config's
    # "name" (jit_bridge_56, jit_bridge_77), and a launch name would remap both.
    bridge = Node(
        package="domain_bridge", executable="domain_bridge",
        arguments=[
            "--from", LaunchConfiguration("shared_domain"),
            "--to", LaunchConfiguration("private_domain"),
            LaunchConfiguration("bridge_config"),
        ],
        output="screen", respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

    return LaunchDescription(args + [OpaqueFunction(function=check_domain), bridge])
