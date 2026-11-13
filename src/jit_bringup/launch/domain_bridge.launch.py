#!/usr/bin/env python3
"""Bridge the sub's private DDS domain to the shared one. Start it FIRST.

The sub runs three launches, each in its own terminal, in this order:

    ros2 launch jit_bringup domain_bridge.launch.py   # 1
    ros2 launch jit_bringup mavros.launch.py          # 2
    ros2 launch jit_bringup jit_bringup.launch.py     # 3

config/domain_bridge.yaml holds all of it:

- ``from_domain`` (56): the shared domain of the boat and the other robots.
- ``to_domain`` (77): the sub's private domain.
- ``topics``: what the bridge carries across, and in which direction.
- ``network.allowed_addresses``: the only addresses DDS may use (loopback and
  eth0), so nothing goes out on wlan0.

Each of the three launches applies the domain and the address restriction to
its own processes (jit_network.py), so no export is needed in any terminal.

Run only one bridge: two bridges pass /tf_static back and forth between each
other. Needs ``sudo apt install ros-humble-domain-bridge``.
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

RESPAWN_DELAY_S = 2.0


def generate_launch_description():
    arg_config = DeclareLaunchArgument(
        "bridge_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("jit_bringup"), "config", "domain_bridge.yaml"]),
        description="Domains, bridged topics and allowed network addresses.")

    # No name=: the process holds one node per domain, named from the config's
    # "name" (jit_bridge_56, jit_bridge_77), and a launch name would remap both.
    # The domains come from the YAML.
    bridge = Node(
        package="domain_bridge", executable="domain_bridge",
        arguments=[LaunchConfiguration("bridge_config")],
        output="screen", respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

    return LaunchDescription([arg_config, network_setup("bridge"), bridge])
