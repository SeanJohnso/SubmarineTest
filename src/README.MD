# JIT Submarine — ROS 2 stack

Packages:

| package | contents |
|---|---|
| `jit_msgs` | `Health`, `Mode`, `ModeRequest`, `LedCommand`, `EstopStatus`, `WaypointXY`, `MissionXY`, `TaskWaypoint`, `TaskMission` |
| `jit_safety` | `crsf_channel_node`, `gpio_estop_node`, `system_monitor_node` |
| `jit_control` | `vehicle_interface_node`, `manual_control_node`, `local_guided_node` |
| `jit_task` | `waypoint_listener_node` — missions from another vessel; `task_node` — the pipeline survey sequence |
| `robotx_comm` | messages shared with the boat (`Heartbeat`, `PipeLineSurveyReport`, …). `heartbeat_node` is no longer launched here — the boat publishes the heartbeat |
| `jit_ui` | `led_driver_node` |
| `jit_bringup` | launch files and config only, no source |
| `ping360_sonar`, `ping360_sonar_msgs` | upstream Ping360 driver, a **git submodule** at `src/ping360_sonar` |

This builds on the **companion Pi**, not on a dev machine — `jit_safety` needs
two system libraries that only exist on the vehicle (below).

---

## External dependencies

### xcrsf — CRSF/ELRS receiver decoding

<https://github.com/Vinx7a/xcrsf>

Used only by `jit_safety/crsf_channel_node`. Installed system-wide with its own
`make install` (it is **not** a ROS package and is not colcon-built), so
`CMakeLists.txt` locates it manually with `find_library`/`find_path`.

```bash
git clone https://github.com/Vinx7a/xcrsf.git
cd xcrsf
# follow the project's own build instructions, then:
sudo make install
```

Check:

```bash
ls /usr/local/include/xcrsf/crossfire.h
ldconfig -p | grep xcrsf
```

API we depend on (`xcrsf/crossfire.h`):

```cpp
explicit XCrossfire(const std::string& uart_path, speed_t baud_rate = 420000);
[[nodiscard]] bool open_port();
bool close_port();
[[nodiscard]] bool is_paired() const;
CRSFLink get_link_state();          // uplink_link_quality, uplink_rssi_antenna_1/2,
                                    // uplink_snr, rf_Mode, tx_power, downlink_*
std::array<uint16_t, 16> get_channel_state();
```

> **KNOWN LIMITATION — `is_paired()` is not a transmitter-presence signal.**
>
> `open_port()` sets the paired flag `true` immediately on a successful UART
> open, before any frame has been decoded. The only thing that clears it on
> link loss is a 250 ms no-RC-frame timeout, and that check runs *inside the
> receive loop* — there is no separate watchdog thread.
>
> Consequence for us: `crsf_channel_node` recreates the `XCrossfire` object
> once a second during a sustained loss, and each `open_port()` re-arms the
> flag. The link-loss latch can therefore fail to engage, and
> `jit/crsf/link_ok` can stay `true` with the transmitter switched off.
>
> `get_link_state()` does **not** rescue this: `link_state_` is only written
> when a LINK_STATISTICS frame is parsed, so with the Tx off it holds its last
> value rather than decaying — a stale `uplink_link_quality` reads as healthy.
>
> A frame-freshness signal is needed. See the TODO in
> `jit_safety/src/crsf_channel_node.cpp::link_healthy()`.

### libgpiod — relay drive and feedback

Used only by `jit_safety/gpio_estop_node`.

```bash
sudo apt update
sudo apt install libgpiod-dev -y
```

Check:

```bash
dpkg -L libgpiod-dev | grep libgpiod
```

> **Version matters.** `GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN` needs
> libgpiod **1.5+**. Without it the relay-feedback pin (BCM18) is requested
> with **no bias**, and a disconnected feedback wire floats instead of reading
> LOW. The node warns at startup when this happens. If you cannot upgrade,
> fit an external ~100k pull-down on that pin.

### rpi_ws281x — LED indicator panel

Used only by `jit_ui/led_driver_node`. Drives the addressable panel over one
GPIO pin via PWM + DMA.

<https://github.com/jgarff/rpi_ws281x>

```bash
git clone https://github.com/jgarff/rpi_ws281x.git
cd rpi_ws281x
mkdir build && cd build
cmake -D BUILD_SHARED=ON ..
make
sudo make install
sudo ldconfig
```

Check:

```bash
ldconfig -p | grep ws2811
```

Needs root (or the right capabilities) to access the PWM/DMA hardware, and
only one process may hold a given DMA channel at a time — do not run
`led_driver_node` alongside another process that also drives the panel.

### yaml-cpp — mission file parsing

Used by `jit_control/local_guided_node`, which parses `waypoints.yaml`
directly because ROS 2 parameters cannot express a list of maps.

```bash
sudo apt install libyaml-cpp-dev -y
```

### ping360_sonar — Ping360 scanning sonar

<https://github.com/CentraleNantesRobotics/ping360_sonar> (`ros2` branch)

Checked in as a git submodule at `src/ping360_sonar`, which itself contains a
submodule (`ping-cpp`). Clone this repo with `--recursive`, and after a pull
that moves the submodule, run:

```bash
git submodule update --init --recursive
```

An empty `src/ping360_sonar/` means this step was skipped.

We run the **C++** node, `ping360_node`. Its dependencies:

```bash
sudo apt install libboost-atomic-dev libboost-program-options-dev \
     libboost-system-dev libboost-thread-dev \
     ros-humble-image-transport ros-humble-image-transport-plugins
# serial access to /dev/ttyUSB0 - log out and back in afterwards
sudo usermod -aG dialout $USER
```

> **`colcon.meta` turns the C++ node on.** Upstream's CMakeLists has
> `option(COMPILE_CPP ON)`, which CMake reads as a help string of `ON` with
> the option **defaulting to OFF**. Without the repo-root `colcon.meta` only
> the Python node is built, and that one needs the pip-only `bluerobotics-ping`.
> colcon only picks up `colcon.meta` from the directory you build in, so build
> from `~/SubmarineTest`. Check: `ls install/ping360_sonar/lib/ping360_sonar/ping360_node`.

> **`fallback_emulated: false` is deliberate.** Upstream defaults it to `true`,
> which publishes random emulated data whenever the sonar cannot be opened.
> Here the node exits with `Cannot initialize sonar`, and the launch respawns
> it until the sonar appears.

Topics, under `jit/sonar/`: `scan_image` (`sensor_msgs/Image`, plus
`/compressed`), `scan` (`sensor_msgs/LaserScan`, once per sweep), and
`scan_echo` (`ping360_sonar_msgs/SonarEcho`). `scan` and `scan_echo` are only
published while something is subscribed. The node pings every 120 ms, so a
full 360° sweep at `angle_step: 1` takes about 48 s. Narrow `angle_sector` or
raise `angle_step` for faster updates.

### ROS 2 packages

`rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `mavros_msgs`, `tf2`, `tf2_ros`.
MAVROS itself is a runtime dependency — see below.

---

## Build

```bash
cd ~/SubmarineTest
colcon build --symlink-install
source install/setup.bash
```

**Use `--symlink-install`.** Without it, `install(DIRECTORY config ...)` copies
`jit_params.yaml` and `waypoints.yaml` at build time, and editing the files
under `src/` has no effect until you rebuild.

**On a memory-limited Pi**, build serially or the compiler gets OOM-killed
(`c++: fatal error: Killed signal terminated program cc1plus`):

```bash
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential
```

---

## Run

```bash
ros2 launch jit_bringup jit_bringup.launch.py
```

Useful arguments:

| argument | default | effect |
|---|---|---|
| `enable_control` | `true` | `false` runs the safety stack only |
| `boot_selftest` | `true` | `false` skips the welded-relay check — bench only |
| `params_file` | package config | override the parameter file |
| `waypoint_file` | package config | override the mission file |
| `enable_sonar` | `true` | `false` leaves out the Ping360 driver |
| `waypoint_source` | `file` | `topic` takes missions from another vessel on `/rviz/waypoints`; `task` holds in AUTO, waits for a list, then runs the pipeline survey (see below) |
| `record_bag` | `true` | with `waypoint_source:=task`, records sonar and navigation topics to `~/jit_bags/` |

Every topic the stack owns is `jit/<name>` (`jit/health`, `jit/crsf/link_ok`,
`jit/cmd/local_guided/setpoint`, …) so the sub can share a DDS domain with
another vessel. MAVROS keeps `/mavros/*`; `/tf`, `/tf_static` and `/rosout`
stay global.

### The pipeline survey (`waypoint_source:=task`)

With the switch in AUTO the sub arms and holds position. When a list arrives on
`/rviz/waypoints`, `task_node` runs this sequence with no further switch input:

1. Follow the list on the surface.
2. Settle at the last waypoint, then dive in place to `survey_depth_m`.
3. Traverse `survey_length_m` along the heading of the list's **last pose**.
4. Rise in place and hold station until the switch leaves AUTO.

The last pose's orientation sets that heading, as yaw counter-clockwise from
East. A zero-length or non-finite quaternion rejects the whole list, with
`waypoint_source:=topic` too. An orientation the sender never set defaults to
yaw 0 and is accepted as East, so check the heading logged by
`waypoint_listener_node`. Depth comes only from `task_node`'s parameters in
`jit_params.yaml`.

`jit/task/active` is true from the moment the list is taken until the sub has
stabilized on the surface. `jit/task/state` (latched) reads `DONE` at the end,
or `FAILED` if a list did not run to completion.

`report_delay_s` (15 s) after either, `task_node` publishes one
`robotx_comm/PipeLineSurveyReport` on `/robotx_comm/jit/tx_pipeline`, reliable and
latched, with `report_segments` (default `1 1 2 1`, a guess) and the sub's GPS
position as the buoy position.

### Missions from another vessel (`waypoint_source:=topic`)

The other vessel publishes:

- `/rviz/waypoints` — `nav_msgs/Path`, `header.frame_id: map`, x/y per pose.
- `/tf_static` — `map → local_origin`, placing the sub's EKF origin in its map.

`waypoint_listener_node` drops every z, transforms the list into
`local_origin` once, and stages it on `jit/nav/local/mission`. It reports
`accepted N` or `rejected: <reason>` on `jit/comms/waypoints/status`. A list is
rejected whole if it is in the wrong frame, has no transform, holds more than
`max_waypoints` (50), puts any waypoint more than `max_waypoint_distance_m`
(20 m) from the sub, or arrives while a mission is RUNNING.

`local_guided_node` flies the staged list at the next entry to LOCAL_GUIDED
(ch8 UP), at the depth it held on entry. **Topic missions never dive** — only
`waypoint_file` can command depth. Another vessel can stage a mission but
cannot start one: that always takes the switch.

MAVROS's `local_position` must stamp its pose with `frame_id: local_origin`,
or every list is rejected.

MAVROS must be running separately:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

Note the vehicle's `MAV_GCS_SYSID` (`SYSID_MYGCS` before ArduPilot 4.6) has to
match MAVROS's `system_id`, or arming and mode changes are accepted at the
MAVLink layer and then silently ignored.

---

## Hardware pins

| signal | pin | direction | notes |
|---|---|---|---|
| relay coil drive | BCM23 / header 16 | output | HIGH = coil energized = motor rail live |
| relay feedback | BCM18 / header 12 | input | 3.3 V LDO on the switched rail. HIGH = rail live |
| CRSF receiver | `/dev/ttyAMA0` | RX only | one-way: the Pi's UART TX is not wired to the Rx |
| LED panel data | BCM13 / header 33 | output | PWM channel 1. Single-wire addressable panel (rpi_ws281x) |
| Ping360 sonar | USB, `/dev/ttyUSB0` | serial | BLU/USB adapter, 115200 baud. Set in `jit_params.yaml` |
