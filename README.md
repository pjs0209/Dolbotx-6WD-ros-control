```md
# DolbotX 6WD ROS2 Control (Real Robot Control Stack)

> ROS2 real-robot control stack for a 6-wheel differential-drive platform (DolbotX family).  
> Handles the full pipeline from high-level outputs → wheel velocity → Arduino motor controller → physical robot.

This repository contains only the **real-robot control / serial bridge / LED control / follower control** portion extracted from the full DolbotX project.

---

# 📸 Media

## Robot
![robot](docs/images/robot_photo_1.jpg)

## Award
![award](docs/images/award_photo.jpg)

---

# 🎥 Videos

- Rough terrain driving: (add link or file)
- Flat terrain driving: (add link or file)
- Multi-area driving: https://www.youtube.com/watch?v=69BXtWKU-2o

---

# 🧠 System Overview

This repository is responsible for **ROS2 → real robot actuation**.

```

Perception / Decision
↓
steering_to_diff
↓
wheel_serial_bridge
↓
Arduino Mega (motor controller)
↓
Physical robot motion

```

Additional pipelines:

```

Vision result → led_serial_bridge → LED Arduino
Joystick → serial_bridge → wheel control
3D target → object_follower → cmd_vel

```

---

# 📂 Repository Structure

```

Arduino/
├── DolbotX_Wheel_Control/
├── LED_Control/
├── LEFT_MOTOR_FINAL/
└── RIGHT_MOTOR_FINAL/

src/
├── serial_bridge/
├── wheel_serial_bridge/
├── steering_to_diff/
├── object_follower/
└── led_serial_bridge/

docs/images/
├── robot_photo_1.jpg
├── award_photo.jpg
├── ros_graph.png
├── led_node_graph.png
└── control_node_graph.png

```

---

# 🧩 ROS Graph

## Full Graph
![graph](docs/images/ros_graph.png)

## LED Graph
![led_graph](docs/images/led_node_graph.png)

## Control Graph
![control_graph](docs/images/control_node_graph.png)

---

# ⚙️ Packages

## steering_to_diff
Converts steering angle to left/right differential wheel velocity.

Input:
```

/angle

```

Output:
```

/left_wheel_speed
/right_wheel_speed

```

---

## wheel_serial_bridge
Sends differential wheel velocities to Arduino Mega via serial.

Input:
```

/left_wheel_speed
/right_wheel_speed

```

Output:
```

Serial → /dev/ttyUSB*

```

---

## serial_bridge
Joystick `/cmd_vel` → differential wheel command → serial.

Input:
```

/cmd_vel

```

---

## object_follower
Generates velocity command to follow a 3D target.

Input:
```

target position

```

Output:
```

/cmd_vel

```

---

## led_serial_bridge
Sends vision result to LED Arduino.

Serial messages:
```

enemy
roka
none

````

---

# 🔧 Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
````

---

# 🚀 Run

## Core control pipeline

```bash
ros2 launch steering_to_diff steering_to_diff.launch.py
ros2 launch wheel_serial_bridge bridge.launch.py
ros2 run led_serial_bridge led_serial_bridge
```

## Teleop

```bash
ros2 run serial_bridge serial_bridge_node
```

## Object follower

```bash
ros2 launch object_follower object_follower.launch.py
```

---

# 🔌 Serial Setup

Check ports:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Grant permission:

```bash
sudo usermod -aG dialout $USER
```

Re-login required.

---

# 🧠 Arduino Firmware

```
Arduino/DolbotX_Wheel_Control
Arduino/LED_Control
```

Upload each sketch using Arduino IDE.

---

# 🛠 Debug

Check topics:

```bash
ros2 topic list
ros2 topic echo /angle
```

Graph:

```bash
rqt_graph
```

---

# 🏁 Competition Use

This stack was used in a real-robot competition environment.

* 6WD differential drive
* real-time serial motor control
* LED state feedback
* target following control

---

# 📜 License

MIT

```
::contentReference[oaicite:0]{index=0}
```

