# JIT Submarine — ROS 2 stack

Packages:

| package | contents |
|---|---|
| `jit_msgs` | `Health`, `Mode`, `ModeRequest`, `LedCommand`, `EstopStatus` |
| `jit_safety` | `crsf_channel_node`, `gpio_estop_node`, `system_monitor_node` |
| `jit_control` | `vehicle_interface_node`, `manual_control_node`, `local_guided_node`, `global_guided_node` |
| `jit_ui` | `led_driver_node` |
| `jit_bringup` | launch files and config only, no source |

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
> `crsf/link_ok` can stay `true` with the transmitter switched off.
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

### ROS 2 packages

`rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `mavros_msgs`.
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
