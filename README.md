
# Dolbotx 6WD ROS2 Control

A modular ROS2 + Arduino control stack for a 6-wheel robot platform.

This repository provides the core “robot base” modules: serial communication, wheel control bridging, LED signaling, and command conversion utilities (steering/twist → differential drive), plus an object-following behavior node.

**Designed for**

* Open-source reuse (clean, modular packages)
* Portfolio demonstration (real robot integration)
* Collaboration (clear build/run/troubleshooting docs)

---

## Table of Contents

* [Key Features](#key-features)
* [System Architecture](#system-architecture)
* [Repository Layout](#repository-layout)
* [Packages](#packages)
* [Getting Started](#getting-started)
* [Build](#build)
* [Run](#run)
* [Arduino Firmware](#arduino-firmware)
* [Serial Port Setup](#serial-port-setup)
* [Troubleshooting](#troubleshooting)
* [Development Notes](#development-notes)
* [Roadmap](#roadmap)
* [License & Attribution](#license--attribution)

---

## Key Features

* **Serial bridge** between ROS2 and microcontroller
* **Wheel motor control bridge** (Arduino-based low-level control)
* **LED status bridge** (simple command-based status signaling)
* **Command conversion utilities**

  * `steering_to_diff`: steering command → left/right wheel velocities
  * `twist_to_diff`: `/cmd_vel` (geometry_msgs/Twist) → differential command
* **Object follower** node for basic target-following behavior
* **ROS2 workspace friendly**: clean packages, `colcon build --symlink-install` ready

---

## System Architecture

Typical deployment pattern:

* **Host PC / Laptop**

  * High-level autonomy logic and ROS2 orchestration
  * Visualization / logging (RViz, rosbag)
* **Microcontroller (Arduino)**

  * Low-level wheel actuation loop (and optional LED control)
* **ROS2 Nodes**

  * Convert and route commands between perception/control and microcontroller bridges

This “distributed robot control” approach is common for real robots (especially when microcontrollers handle realtime-ish actuation). DolbotX describes a similar ROS2 + Jetson + Arduino distributed setup and robust serial bridges for wheels/LED. ([GitHub][1])

---

## Repository Layout

```
Arduino/
  DolbotX_Wheel_Control/
  LED_Control/
  LEFT_MOTER_FINAL/
  RIGHT_MOTOR_FINAL/
src/
  serial_bridge/
  wheel_serial_bridge/
  led_serial_bridge/
  steering_to_diff/
  twist_to_diff/
  object_follower/
```

---

## Packages

### `serial_bridge`

Serial communication foundation package for ROS2 ↔ microcontroller messaging.

### `wheel_serial_bridge`

ROS2 bridge to the Arduino wheel controller (sends target commands, receives status if supported).

### `led_serial_bridge`

ROS2 bridge to an Arduino LED controller (simple “status command” interface).

### `steering_to_diff`

Converts steering-style commands into left/right wheel velocity commands.

### `twist_to_diff`

Converts ROS2 standard velocity command (`/cmd_vel`) into left/right wheel velocity commands.

### `object_follower`

A behavior node that generates motion commands to follow a detected target (implementation depends on your internal message interfaces and perception pipeline).

---

## Getting Started

### Prerequisites

* Ubuntu 22.04
* ROS2 Humble
* Python 3.10+
* `colcon` build tool
* Arduino IDE (for firmware upload)

**Required Ubuntu packages (recommended minimal set):**

* `python3-serial` (pyserial for system Python)

Install:

```bash
sudo apt-get update
sudo apt-get install -y python3-serial
```

> DolbotX also installs additional packages (e.g., RealSense, tf helpers) for their full perception stack. This repo focuses on base control modules only. ([GitHub][1])

---

## Build

Create a workspace and clone:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/pjs0209/Dolbotx-6WD-ros-control.git
```

Install dependencies (best effort):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
```

Build (recommended for Python dev):

```bash
colcon build --symlink-install
source install/setup.bash
```

Verify packages:

```bash
ros2 pkg list | grep -E "serial_bridge|wheel_serial_bridge|led_serial_bridge|steering_to_diff|twist_to_diff|object_follower"
```

---

## Run

> Run nodes in separate terminals (recommended). DolbotX also follows a “multi-terminal modular run” style, which is very maintainable for robotics stacks. ([GitHub][1])

### 1) Core base utilities

Steering → differential:

```bash
ros2 launch steering_to_diff steering_to_diff.launch.py
```

Twist → differential:

```bash
ros2 launch twist_to_diff twist_to_diff.launch.py
```

### 2) Microcontroller bridges

Wheel bridge:

```bash
ros2 launch wheel_serial_bridge bridge.launch.py
```

LED bridge:

```bash
ros2 launch led_serial_bridge bridge.launch.py
```

### 3) Serial bridge (example executable)

```bash
ros2 run serial_bridge serial_bridge_node
```

### 4) Inspect what’s running

```bash
ros2 node list
ros2 node info /serial_bridge_node
ros2 topic list
```


---

## Arduino Firmware

Arduino firmware lives in:

* `Arduino/`

Typical workflow:

1. Open Arduino IDE
2. Select the correct board (e.g., Mega for wheels / Uno/Nano for LEDs)
3. Select the correct port
4. Upload the firmware


---

## Serial Port Setup

List serial devices:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Grant permission:

```bash
sudo usermod -aG dialout $USER
```

Then logout/login (or reboot).

---

## Troubleshooting

### 1) `ModuleNotFoundError: No module named 'serial'`

Install pyserial for **system Python**:

```bash
sudo apt-get install -y python3-serial
```

### 2) Conda environment breaks ROS2 Python packages

ROS2 Humble on Ubuntu 22.04 uses system Python 3.10.
If you run nodes inside conda, you may hit metadata/import issues.

**Recommended**

* Use **system Python** for ROS2 nodes
* Use **conda** only for AI/ML/simulation toolchains

(Exactly the issue you hit earlier: pyserial was in conda, but ROS2 node executed with system Python.)

### 3) Serial device permission denied

* Ensure you’re in `dialout` group
* Re-login
* Confirm device name is correct (`/dev/ttyUSB0`, etc.)

---

## Development Notes

* Use `colcon build --symlink-install` during active development.
* Keep nodes small and composable.
* When adding new packages, document:

  * Node name
  * Parameters
  * Published topics
  * Subscribed topics
  * Example launch command

DolbotX’s README is a good example of how detailed “Usage” sections reduce onboarding time for collaborators. ([GitHub][1])

---

## Roadmap

* Add a **ROS graph** (rqt_graph screenshot) to document node connectivity
* Add **topic/interface tables** per package (publish/subscribe + message types)
* Add CI (lint + build) for collaboration
* Add example bag + playback workflow (optional)
* Add hardware wiring notes (optional)

---

## License & Attribution

This repository was extracted and reorganized from the upstream DolbotX project:

* Upstream: Highsky7/DolbotX ([GitHub][1])

If you are preserving upstream history and code lineage, keep attribution and follow the upstream license terms.

---
