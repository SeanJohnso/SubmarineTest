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

THE LED DRIVER RUNS AS ROOT
---------------------------
rpi_ws281x needs root for the panel's PWM/DMA hardware (/dev/vcio, /dev/mem).
Before any node starts, the launch runs ``sudo -v`` - this is the password
prompt - and then starts ``led_driver_node`` alone under ``sudo -n``. sudo
wipes the environment, LD_LIBRARY_PATH included, so the variables the node
needs are handed across explicitly through ``env``. The node is also pinned to
a UDP-only Fast DDS profile (jit_ui/config/udp_only.xml): shared memory cannot
carry data between a root process and user processes, so without it the node
discovers led/command but never receives a sample. If sudo fails (wrong
password, no terminal) the LED node is skipped and the rest of the stack runs
normally: the panel is indication only and holds no safety policy.

Set ``led_as_root:=false`` to start it as the current user instead.

THE SONAR
---------
``ping360_node`` (upstream ping360_sonar, a git submodule) drives the Ping360
over its USB serial adapter and publishes under ``jit/sonar/``: ``scan_image``,
``scan`` and ``scan_echo``. It is configured with ``fallback_emulated: false``,
so a missing sonar makes the node exit and respawn with an error instead of
quietly publishing emulated data. It holds no safety policy, so its failure
never stops the launch.

Set ``enable_sonar:=false`` to leave it out.

TOPIC NAMES
-----------
Every topic the stack owns is ``jit/<name>``, so this vehicle can share a DDS
domain with another without colliding with it. That is a naming convention,
not access control. MAVROS keeps ``/mavros/*``, and ``/tf``, ``/tf_static`` and
``/rosout`` stay global. ``/robotx_comm/heartbeat`` is also global, because
subscribers on other machines expect that exact name.

WAYPOINT SOURCE
---------------
``waypoint_source:=file`` (default) flies ``waypoint_file`` as always.
``waypoint_source:=topic`` also starts ``waypoint_listener_node``, which takes a
``nav_msgs/Path`` on ``jit/waypoints`` from another vessel, transforms it into
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
    ros2 launch jit_bringup jit_bringup.launch.py led_as_root:=false
    ros2 launch jit_bringup jit_bringup.launch.py enable_sonar:=false
    ros2 launch jit_bringup jit_bringup.launch.py waypoint_source:=task
    ros2 launch jit_bringup jit_bringup.launch.py waypoint_source:=task enable_survey:=true

MAVROS is started separately, with jit_bringup's mavros.launch.py.
"""

import os
import shlex
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
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

# Carried across sudo to the root-run led_driver_node. sudo resets the
# environment and always strips LD_LIBRARY_PATH, without which the node cannot
# find the ROS libraries; the DDS variables keep it on the same ROS graph.
ROOT_ENV_KEEP = (
    "LD_LIBRARY_PATH",
    "AMENT_PREFIX_PATH",
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "RMW_IMPLEMENTATION",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "CYCLONEDDS_URI",
    "ROS_LOG_DIR",
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
        description="Mission source for local_guided_node: waypoint_file, jit/waypoints "
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
    arg_led_root = DeclareLaunchArgument(
        "led_as_root",
        default_value="true",
        description="Start led_driver_node through sudo (asks for the password once).",
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
    def led_driver(context):
        """Start led_driver_node, through sudo unless led_as_root is false.

        Runs as an OpaqueFunction placed ahead of every node, so the password
        prompt appears on a quiet terminal before anything else has started.
        """
        kw = dict(respawn=True, respawn_delay=RESPAWN_DELAY_S)
        as_root = LaunchConfiguration("led_as_root").perform(context).lower()
        if as_root not in ("true", "1", "yes"):
            return [node("jit_ui", "led_driver_node", **kw)]

        print(
            "[bringup] led_driver_node needs root for the LED panel "
            "(rpi_ws281x PWM/DMA) - sudo may ask for your password.",
            flush=True,
        )
        if subprocess.run(["sudo", "-v"]).returncode != 0:
            return [LogInfo(
                msg="[bringup] sudo failed - led_driver_node NOT started, the panel "
                    "stays dark. The rest of the stack is unaffected."
            )]

        # -n: never prompt. The credentials cached by sudo -v above cover the
        # first start and respawns inside sudo's timeout; outside it, sudo
        # fails loudly instead of hanging on a prompt buried in node output.
        forwarded = ["{}={}".format(k, os.environ[k]) for k in ROOT_ENV_KEEP if k in os.environ]
        # Last on the env line, so it wins over any profile forwarded above.
        udp_profile = os.path.join(get_package_share_directory("jit_ui"), "config", "udp_only.xml")
        forwarded.append("FASTRTPS_DEFAULT_PROFILES_FILE={}".format(udp_profile))
        prefix = " ".join(shlex.quote(a) for a in ["sudo", "-n", "/usr/bin/env", *forwarded])
        return [node("jit_ui", "led_driver_node", prefix=prefix, **kw)]

    led_node = OpaqueFunction(function=led_driver)

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

    # Always on. Read-only: it reports status and cannot move the vehicle.
    heartbeat_node = node(
        "robotx_comm", "heartbeat_node",
        respawn=True, respawn_delay=RESPAWN_DELAY_S,
    )

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
        arg_params,
        arg_waypoints,
        arg_wp_source,
        arg_survey,
        arg_record_bag,
        arg_control,
        arg_selftest,
        arg_led_root,
        arg_sonar,
        # First, so the sudo prompt comes before any node output.
        led_node,
        LogInfo(msg="[bringup] starting the JIT stack - health gate does the sequencing"),
        abort_on_estop_failure,
        crsf_node,
        gpio_node,
        monitor_node,
        vehicle_node,
        manual_node,
        local_node,
        listener_node,
        task_node,
        heartbeat_node,
        sonar_node,
        task_bag_action,
    ])
