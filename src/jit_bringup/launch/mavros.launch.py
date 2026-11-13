#!/usr/bin/env python3
"""MAVROS for the JIT submarine.

The same node as the stock ``mavros apm.launch``, with the stock
``apm_config.yaml`` read from the installed MAVROS and then
``config/mavros_params.yaml`` loaded last so that it wins. That one file holds
all of the sub's MAVROS settings:

- The plugin list. Only the plugins the stack uses are loaded, to cut CPU use
  on the Pi. It replaces the stock ``apm_pluginlists.yaml``, which is not loaded.
- The local position frame, renamed from MAVROS's default ``map`` to
  ``jit/local_origin``, because the boat's frame is also called ``map``. Two
  frames cannot share a name in one TF tree, and the boat publishes
  ``map`` -> ``jit/local_origin``.

Second of the sub's three launches, after domain_bridge.launch.py. It sets
ROS_DOMAIN_ID and the DDS address restriction from config/domain_bridge.yaml
(jit_network.py), so MAVROS is on the sub's private domain and off wlan0.

Usage:
    ros2 launch jit_bringup mavros.launch.py
    ros2 launch jit_bringup mavros.launch.py fcu_url:=udp://:14550@<blueos-ip>:14560
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from jit_network import network_setup  # noqa: E402


def generate_launch_description():
    mavros_share = FindPackageShare("mavros")
    args = [
        DeclareLaunchArgument(
            "fcu_url", default_value="udp://:14550@201.7.90.195:14560",
            description="MAVLink endpoint. The default is the BlueOS router on the vehicle."),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        DeclareLaunchArgument("tgt_component", default_value="1"),
        DeclareLaunchArgument("fcu_protocol", default_value="v2.0"),
    ]

    mavros = Node(
        package="mavros", executable="mavros_node", namespace="mavros", output="screen",
        parameters=[
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": LaunchConfiguration("tgt_system"),
                "tgt_component": LaunchConfiguration("tgt_component"),
                "fcu_protocol": LaunchConfiguration("fcu_protocol"),
            },
            PathJoinSubstitution([mavros_share, "launch", "apm_config.yaml"]),
            PathJoinSubstitution(
                [FindPackageShare("jit_bringup"), "config", "mavros_params.yaml"]),
        ],
    )

    return LaunchDescription(args + [network_setup("mavros"), mavros])
