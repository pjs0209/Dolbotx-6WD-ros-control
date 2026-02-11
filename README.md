Dolbotx 6WD ROS2 Control Modules

This repository contains ROS2 and Arduino modules for controlling a 6WD robot platform.
The code was extracted from the original DolbotX project and reorganized as a standalone repository for development, collaboration, and open-source use.

This repository is intended for:

Open-source release

Portfolio demonstration

Collaborative robotics development

Repository Structure
Arduino/
src/
  serial_bridge/
  wheel_serial_bridge/
  led_serial_bridge/
  steering_to_diff/
  object_follower/
Packages
Package	Description
serial_bridge	Serial communication bridge between ROS2 and microcontroller
wheel_serial_bridge	Wheel motor serial control interface
led_serial_bridge	LED control via serial
steering_to_diff	Convert steering commands to differential drive
object_follower	Object tracking and motion generation
System Requirements

Ubuntu 22.04

ROS2 Humble

Python 3.10

Arduino IDE

pyserial

Install dependency:

sudo apt install python3-serial
Build

Create workspace and clone:

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/pjs0209/Dolbotx-6WD-ros-control.git

Build:

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash

Verify packages:

ros2 pkg list | grep -E "serial_bridge|wheel_serial_bridge|led_serial_bridge|steering_to_diff|object_follower"
Running Nodes

Example:

ros2 run serial_bridge serial_bridge_node

Check node info:

ros2 node list
ros2 node info /serial_bridge_node
Serial Port Setup

Find device:

ls /dev/ttyUSB*

Fix permission:

sudo usermod -aG dialout $USER

Logout/login after.

Arduino

The Arduino/ folder contains firmware for the microcontroller.

Upload using Arduino IDE and set:

correct port

correct baudrate

Development Notes

If using conda environments, ROS2 nodes may fail to detect Python packages.
Recommended approach:

Use system Python for ROS2 nodes

Use conda for AI / RL / simulation only

License

This repository contains code extracted from the original DolbotX project.

Upstream:
https://github.com/Highsky7/DolbotX

See LICENSE file for details.

Author

Junseong Park
Robotics / ROS2 / Control Systems

This repository is maintained as part of robotics research and development work.
