
---

# DolbotX 6WD ROS2 Control

Real-world ROS2 control stack for a 6-wheel autonomous robot platform.
Designed for research deployment, robotics competitions, and modular open-source reuse.

This repository contains the **actual control system used on a physical robot**, including perception-driven steering, differential drive conversion, and serial motor control.

---

## Demo

### Multi-area autonomous driving (YouTube Short)

[https://www.youtube.com/shorts/69BXtWKU-2o](https://www.youtube.com/shorts/69BXtWKU-2o)

This demo shows:

* Vision-based drivable area detection
* Steering command generation
* Differential wheel control
* Real terrain driving

---

## Real Robot

<img src="docs/robot_photo_1.jpg" width="600"/>

<img src="docs/award_photo.jpg" width="600"/>

Used in real competition environment and field testing.

---

## System Architecture

High-level perception and control run on the host PC.
Low-level motor control runs on Arduino.

Pipeline:

camera → perception → steering angle → diff conversion → serial bridge → motor controller

This distributed structure is typical for real robotic systems where microcontrollers handle motor loops.

---

## ROS Graph

<img src="docs/ros_graph.png" width="900"/>

The graph above shows the actual running node structure.

Key flow:

1. Camera publishes image topics
2. Vision node computes drivable area
3. Steering angle generated
4. steering_to_diff converts to wheel velocities
5. wheel_serial_bridge sends to Arduino
6. Motors execute

---

## Repository Structure

```
Arduino/
 ├── DolbotX_Wheel_Control
 ├── LEFT_MOTOR_FINAL
 ├── RIGHT_MOTOR_FINAL
 └── LED_Control

src/
 ├── serial_bridge
 ├── wheel_serial_bridge
 ├── steering_to_diff
 ├── object_follower
 └── led_serial_bridge

docs/
 ├── ros_graph.png
 ├── robot_photo_1.jpg
 └── award_photo.jpg
```

---

## Packages

### serial_bridge

Base serial communication node between ROS2 and microcontrollers.

### wheel_serial_bridge

Sends wheel velocity commands to Arduino motor controller.

### steering_to_diff

Converts steering angle → left/right wheel velocities.

### object_follower

Generates motion commands based on detected targets.

### led_serial_bridge

Controls LED status via serial.

---

## Hardware Architecture

Host PC / Jetson

* ROS2 nodes
* perception
* control

Arduino

* motor control
* LED control

Connections:

* USB serial → Arduino
* Camera → ROS2
* Arduino → motor drivers

---

## Build

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Run

### Core control pipeline

```
ros2 launch steering_to_diff steering_to_diff.launch.py
ros2 launch wheel_serial_bridge bridge.launch.py
ros2 run serial_bridge serial_bridge_node
ros2 run led_serial_bridge led_serial_bridge
```

---

## Launch All

Create `launch_all.sh` in repo root:

```
#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch steering_to_diff steering_to_diff.launch.py &
ros2 launch wheel_serial_bridge bridge.launch.py &
ros2 run serial_bridge serial_bridge_node &
ros2 run led_serial_bridge led_serial_bridge &

wait
```

Run:

```
chmod +x launch_all.sh
./launch_all.sh
```

---

## Arduino Firmware

Firmware located in:

```
Arduino/
```

Upload via Arduino IDE:

* Select board
* Select port
* Upload

---

## Serial Setup

```
ls /dev/ttyUSB* /dev/ttyACM*
```

Permission:

```
sudo usermod -aG dialout $USER
```

Re-login required.

---

## Development Notes

* Ubuntu 22.04
* ROS2 Humble
* Python 3.10
* Use system Python for ROS2 nodes
* Virtualenv may break rclpy imports

---

## Research Context

Developed for real robotic deployment including:

* Rough terrain navigation
* Vision-based driving
* Autonomous robotics competition
* Modular ROS2 architecture

The system has been validated on physical hardware.

---

## Intended Use

This repository is maintained for:

* Robotics portfolio
* Research lab submission
* Open-source reuse
* Educational robotics

---

## Roadmap

Planned improvements:

* Hardware wiring diagram
* Full architecture diagram
* Topic interface tables
* Simulation integration
* CI pipeline

---

## Author

Robotics Engineer
ROS2 / Embedded / Autonomous Systems

---

