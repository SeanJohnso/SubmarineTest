#!/usr/bin/env python3
"""Bring-up for the JIT submarine stack.

WHAT CHANGED FROM THE OLD CHAINED LAUNCH
----------------------------------------
The previous launch file started one node, probed its topic, and only then
started the next. That encoded the old series architecture into the bring-up.
With a health bus it is the wrong shape: ordering is ``system_monitor_node``'s
job at runtime, and it already does it. Nothing can arm until ``jit/health``
says so, and until each node has checked in the monitor simply reports which
one it is still waiting on.

So: everything starts at once, everything respawns, and the runtime gate does
the sequencing.

THE ONE HARD GATE
-----------------
``gpio_estop_node`` is started with ``respawn=False`` and a handler that shuts
the whole launch down if it exits non-zero. That node exits non-zero when it
cannot claim its GPIO lines, when it cannot read the relay feedback, or when
the boot self-test finds the relay welded shut. A vehicle whose e-stop does not
work should not finish booting, and respawning it in a loop would just hide the
fault behind a wall of restarts.

Set ``boot_selftest:=false`` for bench work where the relay or the feedback LDO
is not connected.

THE LED DRIVER
--------------
``led_driver_node`` drives the panel through SPI0 (/dev/spidev0.0) and runs as
the login user like every other node: no sudo. The user must be in the group
that owns /dev/spidev0.0 and /dev/gpiomem (dialout on the Pi). The panel is
indication only and holds no safety policy.

THE SONAR
---------
``ping360_node`` (upstream ping360_sonar, a git submodule) drives the Ping360
over its USB serial adapter and publishes under ``jit/sonar/``: ``scan_image``,
``scan`` and ``scan_echo``. It is configured with ``fallback_emulated: false``,
so a missing sonar makes the node exit and respawn with an error instead of
quietly publishing emulated data. It holds no safety policy, so its failure
never stops the launch.

Set ``enable_sonar:=false`` to leave it out.

TOPIC NAMES AND DDS DOMAINS
---------------------------
Every topic the stack owns is ``jit/<name>``. MAVROS keeps ``/mavros/*``, and
``/tf``, ``/tf_static`` and ``/rosout`` stay global. ``/robotx_comm/jit/tx_pipeline``
is also global, because subscribers on other machines expect that exact name.
The heartbeat is published by the boat, not by this launch.

The sub runs on its own DDS domain, 77, not on the shared one, 56: on the
shared domain MAVROS took minutes to start. This launch, like the other two,
sets ROS_DOMAIN_ID and a Fast DDS address restriction (no wlan0) from
config/domain_bridge.yaml before starting anything - see jit_network.py.

WAYPOINT SOURCE
---------------
``waypoint_source:=file`` (default) flies ``waypoint_file`` as always.
``waypoint_source:=topic`` also starts ``waypoint_listener_node``, which takes a
``nav_msgs/Path`` on ``/rviz/waypoints`` from another vessel, transforms it into
the sub's local frame and stages it for ``local_guided_node``. Topic missions
are surface-only: the depth is never taken from the topic. Either way a mission
only starts on entry to LOCAL_GUIDED.

``waypoint_source:=task`` runs the pipeline survey. It starts
``waypoint_listener_node`` and ``task_node``. The sub arms and holds position on
entry to LOCAL_GUIDED, starts moving as soon as a list arrives, follows it on the
surface and holds station at the end until the switch leaves AUTO. Only with
``enable_survey:=true`` does it dive, traverse and surface first. In this mode
the launch sets ``hold_timeout_s`` to 0 so the final hold never times out, and
records the sonar and navigation topics to ``~/jit_bags/`` unless
``record_bag:=false``.

Usage:
    ros2 launch jit_bringup jit_bringup.launch.py
    ros2 launch jit_bringup jit_bringup.launch.py enable_control:=false
    ros2 launch jit_bringup jit_bringup.launch.py boot_selftest:=false
    ros2 launch jit_bringup jit_bringup.launch.py params_file:=/path/to/my.yaml
    ros2 launch jit_bringup jit_bringup.launch.py enable_sonar:=false
    ros2 launch jit_bringup jit_bringup.launch.py waypoint_source:=task
    ros2 launch jit_bringup jit_bringup.launch.py waypoint_source:=task enable_survey:=true

This is the third of the sub's three launches, each in its own terminal:

    ros2 launch jit_bringup domain_bridge.launch.py   # 1
    ros2 launch jit_bringup mavros.launch.py          # 2
    ros2 launch jit_bringup jit_bringup.launch.py     # 3
"""

import os
import sys
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from jit_network import network_setup  # noqa: E402

RESPAWN_DELAY_S = 2.0

# Recorded for the whole of a waypoint_source:=task run. scan_echo is the raw
# sonar data, and the driver only publishes it while something subscribes.
TASK_BAG_TOPICS = (
    "/jit/sonar/scan_echo",
    "/jit/sonar/scan",
    "/mavros/local_position/pose",
    "/jit/mode",
    "/jit/nav/local/status",
    "/jit/nav/local/submerged",
    "/jit/task/state",
    "/jit/task/active",
)


def generate_launch_description():
    pkg_share = FindPackageShare("jit_bringup")

    arg_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "jit_params.yaml"]),
        description="Node parameters, keyed by node name.",
    )
    arg_waypoints = DeclareLaunchArgument(
        "waypoint_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "waypoints.yaml"]),
        description="Mission file for local_guided_node. Plain YAML, not ROS params.",
    )
    arg_wp_source = DeclareLaunchArgument(
        "waypoint_source",
        default_value="file",
        choices=["file", "topic", "task"],
        description="Mission source for local_guided_node: waypoint_file, /rviz/waypoints "
                    "from another vessel via waypoint_listener_node (surface-only), or "
                    "task: hold in AUTO, wait for a list, then run the pipeline survey.",
    )
    arg_survey = DeclareLaunchArgument(
        "enable_survey",
        default_value="false",
        description="With waypoint_source:=task, dive, traverse and surface after the "
                    "surface path. false = follow the path and hold, never dive.",
    )
    arg_record_bag = DeclareLaunchArgument(
        "record_bag",
        default_value="true",
        description="With waypoint_source:=task, record the sonar and navigation topics "
                    "to ~/jit_bags/.",
    )
    arg_control = DeclareLaunchArgument(
        "enable_control",
        default_value="true",
        description="Start the locomotion and vehicle-I/O nodes. false = safety stack only.",
    )
    arg_selftest = DeclareLaunchArgument(
        "boot_selftest",
        default_value="true",
        description="Run the welded-relay check at startup. Only disable on the bench.",
    )
    arg_sonar = DeclareLaunchArgument(
        "enable_sonar",
        default_value="true",
        description="Start the Ping360 sonar driver.",
    )
    params = LaunchConfiguration("params_file")
    waypoints = LaunchConfiguration("waypoint_file")
    control_on = IfCondition(LaunchConfiguration("enable_control"))

    def node(pkg, exe, extra_params=None, **kw):
        param_list = [params]
        if extra_params:
            param_list.append(extra_params)
        return Node(
            package=pkg, executable=exe, name=exe,
            parameters=param_list, output="screen", **kw
        )

    # --- Safety stack -------------------------------------------------------
    crsf_node = node(
        "jit_safety", "crsf_channel_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

    # No respawn: see the module docstring. This node failing is a stop-work.
    gpio_node = node(
        "jit_safety", "gpio_estop_node",
        # ParameterValue with an explicit type: a bare LaunchConfiguration
        # resolves to the STRING "true"/"false", and the node declares this
        # parameter as a bool, so passing it raw is a type error at runtime.
        extra_params={
            "boot_selftest": ParameterValue(
                LaunchConfiguration("boot_selftest"), value_type=bool
            )
        },
        respawn=False,
    )

    def system_monitor(context):
        """Start system_monitor_node.

        A task run ends by holding station until the operator takes over, so
        the timeout that disarms after a finished mission is turned off for it.
        """
        source = LaunchConfiguration("waypoint_source").perform(context)
        extra = {"hold_timeout_s": 0.0} if source == "task" else None
        return [node(
            "jit_safety", "system_monitor_node", extra_params=extra,
            respawn=True, respawn_delay=RESPAWN_DELAY_S,
        )]

    monitor_node = OpaqueFunction(function=system_monitor)

    # --- Indication ---------------------------------------------------------
    led_node = node(
        "jit_ui", "led_driver_node", respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

    # --- Sensors ------------------------------------------------------------
    # Namespaced, so its jit_params.yaml key is /jit/sonar/ping360_node.
    sonar_node = node(
        "ping360_sonar", "ping360_node", namespace="jit/sonar",
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
        condition=IfCondition(LaunchConfiguration("enable_sonar")),
    )

    # --- Control stack ------------------------------------------------------
    vehicle_node = node(
        "jit_control", "vehicle_interface_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )
    manual_node = node(
        "jit_control", "manual_control_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )
    local_node = node(
        "jit_control", "local_guided_node",
        extra_params={
            "waypoint_file": waypoints,
            "waypoint_source": LaunchConfiguration("waypoint_source"),
        },
        respawn=True, respawn_delay=RESPAWN_DELAY_S, condition=control_on,
    )

    # --- Inter-vessel ---------------------------------------------------------
    # Only when a list can come from another vessel. Holds no safety policy and
    # cannot move the vehicle: the worst it can do is fail to stage a mission.
    listener_node = node(
        "jit_task", "waypoint_listener_node",
        # The boat publishes on /rviz/waypoints. The node's own name for the
        # topic stays jit/waypoints; this remap is the only place it changes.
        remappings=[("jit/waypoints", "/rviz/waypoints")],
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("waypoint_source"), "' in ('topic', 'task')",
        ])),
    )

    # --- Task -----------------------------------------------------------------
    task_node = node(
        "jit_task", "task_node",
        # Typed explicitly: a bare LaunchConfiguration is the string "true".
        extra_params={
            "survey_enabled": ParameterValue(
                LaunchConfiguration("enable_survey"), value_type=bool
            )
        },
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
        condition=LaunchConfigurationEquals("waypoint_source", "task"),
    )

    def task_bag(context):
        """Record a task run. A recorder failure never affects the vehicle."""
        source = LaunchConfiguration("waypoint_source").perform(context)
        record = LaunchConfiguration("record_bag").perform(context).lower()
        if source != "task" or record not in ("true", "1", "yes"):
            return []
        # Outside the workspace, so a run never dirties the git checkout.
        bag_root = os.path.expanduser("~/jit_bags")
        os.makedirs(bag_root, exist_ok=True)
        bag_dir = os.path.join(bag_root, "task_" + time.strftime("%Y%m%d_%H%M%S"))
        return [ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_dir, *TASK_BAG_TOPICS],
            output="screen",
        )]

    task_bag_action = OpaqueFunction(function=task_bag)

    def on_estop_exit(event, context):
        """Abort the launch only on a REAL failure.

        gpio_estop_node returns 1 when it throws: GPIO lines it cannot claim, a
        feedback pin it cannot read, or a welded relay found by the boot
        self-test. Any of those means the e-stop does not work and nothing else
        should be running.

        A clean exit (0) or death by signal (negative returncode) is just the
        launch shutting down - Ctrl-C sends SIGINT to every process, this one
        included. Treating that as a failure printed an alarming and completely
        false "the e-stop is not functional" on every normal Ctrl-C.
        """
        rc = event.returncode
        if rc is None or rc <= 0:
            return [LogInfo(msg="[bringup] gpio_estop_node exited cleanly (rc={}).".format(rc))]
        return [
            LogInfo(
                msg="[bringup] gpio_estop_node FAILED (rc={}) - the e-stop is not "
                    "functional. Shutting the launch down. Check the GPIO lines, the "
                    "relay feedback wiring, and whether the boot self-test found a "
                    "welded relay.".format(rc)
            ),
            EmitEvent(event=Shutdown(reason="gpio_estop_node failed")),
        ]

    abort_on_estop_failure = RegisterEventHandler(
        OnProcessExit(target_action=gpio_node, on_exit=on_estop_exit)
    )

    return LaunchDescription([
        # First, so every process below gets the domain and the DDS profile.
        network_setup("bringup"),
        arg_params,
        arg_waypoints,
        arg_wp_source,
        arg_survey,
        arg_record_bag,
        arg_control,
        arg_selftest,
        arg_sonar,
        LogInfo(msg="[bringup] starting the JIT stack - health gate does the sequencing"),
        abort_on_estop_failure,
        led_node,
        crsf_node,
        gpio_node,
        monitor_node,
        vehicle_node,
        manual_node,
        local_node,
        listener_node,
        task_node,
        sonar_node,
        task_bag_action,
    ])
