
```md
# DolbotX 6WD Real-Robot Control Stack (ROS2 ↔ Arduino PID Drive)

> A competition-grade real-robot control stack for a 6-wheel differential drive platform.  
> High-level commands from ROS2 are converted into left/right wheel velocity targets and streamed over serial to an Arduino Mega that runs a **closed-loop PID speed controller** (encoder feedback) and drives all six motors.

---

## ✨ Key Features (Portfolio Highlights)

- **Real robot deployment** (not simulation-only): ROS2 → serial → embedded motor control
- **Closed-loop wheel speed control** using encoder feedback + **PID** (with anti-windup)
- **Deterministic control loop** at **50 Hz** (`CONTROL_MS = 20 ms`)
- **Robust serial parsing** (line-based, non-blocking buffer)
- **Fail-safe behaviors**:
  - stop until first valid command is received
  - (test firmwares) command watchdog timeout
  - low-speed deadzone + minimum PWM suppression
- **LED status subsystem** for competition state feedback (friend/enemy/none)

---

## 📸 Media

### Robot
![robot](docs/images/robot_photo_1.jpg)

### Award
![award](docs/images/award_photo.jpg)

### Videos
- Rough terrain driving: (add link)
- Flat terrain driving: (add link)
- Multi-area autonomous driving (YouTube): https://www.youtube.com/watch?v=69BXtWKU-2o

---

## 🧠 System Architecture

### End-to-End Pipeline

```

Perception / Planner (ROS2)
↓
Wheel velocity targets (m/s)
↓
Serial packet stream (USB)
↓
Arduino Mega (PID speed control @ 50 Hz)
↓
Motor drivers → 6 motors (3 left, 3 right)
↓
Physical robot motion

```

### Hardware Split

- **Host PC / Jetson**: perception + decision + ROS2 nodes
- **Arduino Mega**: motor control loop (encoders + PID + PWM)
- **Arduino (LED)**: simple LED state machine via serial strings

---

## 📂 Repository Structure

```

Arduino/
├── DolbotX_Wheel_Control/     # Main 6WD controller (Mega): VL <L> <R>\n
├── LED_Control/              # LED controller: "roka" / "enemy" / "none"
├── LEFT_MOTOR_FINAL/         # Left-side standalone PID test (VL...)
└── RIGHT_MOTOR_FINAL/        # Right-side standalone PID test (VR...)

src/
├── steering_to_diff/
├── wheel_serial_bridge/
├── serial_bridge/
├── object_follower/
└── led_serial_bridge/

docs/images/
├── ros_graph.png
├── robot_photo_1.jpg
└── award_photo.jpg

```

---

# 🔌 Serial Protocol (Exact)

## 1) Motor Controller (Arduino Mega) — `DolbotX_Wheel_Control.ino`

- **Baudrate**: `57600`
- **Packet format (line-based)**:

```

VL <left_velocity_mps> <right_velocity_mps>\n

```

Example:
```

VL 0.5 -0.5\n

```

- Parsing method:
  - non-blocking char buffer (size 64)
  - newline terminates a command
  - tokens parsed with `strtok()` + `atof()`

✅ A command is considered **valid** only when both velocities are parsed.

---

## 2) LED Controller — `LED_Control.ino`

- **Baudrate**: `115200`
- **Commands** (case-insensitive, whitespace-trimmed):

| Command | Action |
|---|---|
| `roka`  | Green ON, Red OFF |
| `enemy` | Red ON, Green OFF |
| `none` or others | Both OFF |

Line-based:
```

<command>\n

```

---

## 3) Standalone Motor Test Firmwares

### RIGHT — `RIGHT_MOTOR_FINAL.ino`
- **Baudrate**: `57600`
- Accepts:
```

VR<velocity_mps>\n
VR <velocity_mps>\n

```
(`atof(serialBuffer + 2)` tolerates leading spaces)

### LEFT — `LEFT_MOTOR_FINAL.ino`
- **Baudrate**: `57600`
- Accepts:
```

VL<velocity_mps>\n
VL <velocity_mps>\n

````

---

# ⚙️ Arduino Motor Control (Deep Dive)

## A) Motor / Encoder Topology (6WD Differential)

The robot is driven as two motor groups:

- **Left side**: M1 (encoder), M2, M3
- **Right side**: M4 (encoder), M5, M6

Only **one encoder per side** is used in closed-loop feedback:
- left feedback: **M1 encoder**
- right feedback: **M4 encoder**

The remaining motors are driven with the **same control command** as the encoder motor (with direction inversion as needed).

### Direction Mapping (Important)

In main controller (`DolbotX_Wheel_Control.ino`):

- Left:
  - `M1` uses `+cmdL`
  - `M2`, `M3` use `-cmdL` (hardware mounting inverted)

- Right:
  - `M4` uses `+cmdR`
  - `M5`, `M6` use `-cmdR`

---

## B) Deterministic Control Loop Timing

- Timer: `MsTimer2`
- Period: `CONTROL_MS = 20 ms`
- Loop runs when `flag_Control == true`

So control frequency:
\[
f = \frac{1}{0.02} = 50\ \text{Hz}
\]

---

## C) Velocity/RPM Conversion (Exact)

Constants (main controller):

- `WHEEL_DIAM = 0.135 m`
- \[
WHEEL\_CIRC = \pi \cdot WHEEL\_DIAM
\]
- `GEAR_RATIO = 1/71` (motor → wheel)

### 1) Target wheel RPM from target velocity (m/s)

Given target velocity \( v \) [m/s]:
\[
RPM_{target} = \frac{v}{WHEEL\_CIRC} \cdot 60
\]

Implemented as:
```cpp
targetRPM = vel_mps * 60.0 / WHEEL_CIRC;
````

### 2) Measured motor RPM from encoder counts

Encoder counts in one control window (dt):

* `PULSES_PER_REV = 350` (main firmware)
* Measured motor RPM:
  [
  RPM_{motor} = \frac{cnt}{PULSES_PER_REV} \cdot \frac{60}{dt}
  ]

Implemented as:

```cpp
motorRPM = (cnt / (double)PULSES_PER_REV) * (60.0 / dt_s);
```

### 3) Wheel RPM from motor RPM via gear ratio

[
RPM_{wheel} = RPM_{motor} \cdot GEAR_RATIO
]
where `GEAR_RATIO = 1/71`.

---

## D) PID Controller (Exact Equation + Anti-windup)

### PID Equation

Error:
[
e(t) = RPM_{target} - RPM_{wheel}
]

Integral:
[
I(t) = I(t-\Delta t) + e(t)\Delta t
]
with anti-windup clamp:
[
I(t) \in [-I_{max},\ I_{max}]
]
(`I_MAX = 3000`)

Derivative:
[
D(t) = \frac{e(t) - e(t-\Delta t)}{\Delta t}
]

Output command (pre-saturation):
[
u(t) = K_P e(t) + K_I I(t) + K_D D(t)
]

### Main 6WD firmware gains

* `KP = 0.8`
* `KI = 0.0`
* `KD = 0.0`

So in the current main firmware configuration, PID is effectively **P-control** (I/D disabled), but the structure supports full PID.

---

## E) PWM Mapping + Saturation (Exact)

### 1) Command saturation (main firmware)

After PID:

```cpp
cmd = constrain(cmd, -200, 200);
```

So:
[
u \in [-200, 200]
]

### 2) PWM generation (driveMotor)

PWM is computed as:
[
PWM = round(|u|)
]

Then:

* cap:

  * main firmware: `PWM ≤ 200`
  * test firmwares: `PWM ≤ 250`
* deadzone:

  * if `PWM < 15` → `PWM = 0` (prevents weak buzzing / no-motion)

Direction:

* `cmd >= 0` → `DIR = HIGH`
* `cmd < 0` → `DIR = LOW`

---

## F) Low-speed Stop Logic (Exact)

In main firmware:

* if target velocity magnitude is small:

```cpp
if (fabs(vel_left_Sub) <= 0.05) cmdL = 0;
if (fabs(vel_right_Sub) <= 0.05) cmdR = 0;
```

So:

* if (|v| \le 0.05\ m/s) → full stop

---

## G) Safety Behaviors

### Main 6WD firmware

* **No command received yet** → motors stay stopped forever until first valid `VL ...` is received:

```cpp
if (!velReceived) { stop all; return; }
```

### Standalone test firmwares (LEFT/RIGHT)

Additional robustness:

1. **Command watchdog**:

* `CMD_TIMEOUT_MS = 300 ms`
* if timeout → stop all motors

2. **EMA filter on RPM** (noise reduction before PID):

[
RPM_{filt} = \alpha RPM + (1-\alpha) RPM_{filt}
]

* `RPM_ALPHA = 0.3`

3. **Integral reset on new command**:

* prevents windup when velocity steps occur:

```cpp
integral = 0; prevErr = 0;
```

---

# 💡 PID Tuning Notes (Emphasized)

### Why PID matters here

* DC motor speed under load varies significantly (terrain, battery sag, friction).
* A velocity command without feedback becomes inconsistent.
* Encoder-based closed-loop PID ensures wheel speeds track targets reliably even in rough terrain.

### Practical tuning workflow (what I used / recommend)

1. **Start with P-only**

   * Increase (K_P) until you get fast response but not oscillation.

2. **Add I to remove steady-state error**

   * Increase (K_I) gradually.
   * Keep anti-windup clamp (already implemented via `I_MAX`).

3. **Add EMA filtering if encoder RPM is noisy**

   * `RPM_ALPHA` higher → faster response, noisier.
   * `RPM_ALPHA` lower → smoother, slower.

4. Validate using Serial Plotter

* Output:

  * target velocity
  * actual velocity
  * command PWM

(LEFT/RIGHT test codes already include commented Serial plot lines)

---

# 🟦 ROS2 Layer (What must match Arduino)

> The ROS2 nodes must output wheel velocity targets in **m/s**, and the bridge must send the exact serial format required by the Arduino.

### Requirements to be compatible with Arduino main firmware

* Must send:

```
VL <left_mps> <right_mps>\n
```

* Must use serial port at `57600`
* Update fast enough (recommended ≥ 10–20 Hz; control loop is 50 Hz)

### LED bridge requirements

* Must send one of:

  * `roka\n`
  * `enemy\n`
  * `none\n`
    at `115200`

---

## ✅ What I will fill in once ROS code is provided

From `src/steering_to_diff`, `src/wheel_serial_bridge`, `src/serial_bridge`, `src/object_follower`, `src/led_serial_bridge`, I will extract:

* **Exact topic names**
* **Message types**
* **Parameters** (wheelbase, track width, max speed, serial port, baudrate, scaling, etc.)
* **Control equations** used:

  * differential kinematics
  * follower control law (distance/heading)
* Launch structure and node graph

(Arduino side is now fully analyzed and finalized.)

---

# 🔧 Build (ROS2 Humble)

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

# 🚀 Run

Core pipeline (example):

```bash
ros2 launch steering_to_diff steering_to_diff.launch.py
ros2 launch wheel_serial_bridge bridge.launch.py
ros2 run led_serial_bridge led_serial_bridge
```

Teleop (optional):

```bash
ros2 run serial_bridge serial_bridge_node
```

Follower (optional):

```bash
ros2 launch object_follower object_follower.launch.py
```

---

# 🔌 Serial Device Setup

```bash
ls /dev/ttyUSB* /dev/ttyACM*
sudo usermod -aG dialout $USER
```

Re-login required.

---

# 🧪 Troubleshooting

### Motors don’t move

* confirm valid serial command arrives (must end with `\n`)
* verify baudrate `57600`
* check driver enable pins are HIGH (start_pin1~6)
* check deadzone (`PWM < 15` → 0)

### Robot stops unexpectedly (test firmware)

* watchdog timeout is **300 ms**
* ensure commands stream continuously

### Unstable speed / oscillation

* reduce `KP`
* reduce `KI` or increase integral clamp strictness
* lower `RPM_ALPHA` for more filtering

---

# 👤 Author Contribution

* Designed the real-robot ROS2 ↔ Arduino architecture
* Implemented serial command protocol and robust parsing
* Built encoder-based **PID wheel speed control**
* Added field safety behaviors and test firmwares
* Integrated LED feedback subsystem for competition

---

# 📜 License

MIT

```

---

## 다음으로 “ROS 노드 코드 분석(토픽/파라미터/제어식)”까지 **진짜 확정**하려면
너가 원하는 2번(ROS 노드 분석)은 **`src/` 안의 실제 파이썬 파일 내용**이 필요해.  
여기 대화에 아래 파일들만 그대로 붙여줘 (혹은 최소한 주요 노드 파일들):

- `src/wheel_serial_bridge/**` (시리얼 포맷 송신부 핵심)
- `src/steering_to_diff/**` (차동 변환 수식/파라미터 핵심)
- `src/led_serial_bridge/**` (LED 문자열 송신 확인)
- (있으면) `src/object_follower/**`, `src/serial_bridge/**`

그럼 위 README에서 **ROS 섹션을 “정확한 토픽명/타입/파라미터 표 + 제어식 + 런치 플로우 + rqt_graph 기준”**으로 완성해서, Mandol_ws/DolbotX급으로 한 번에 최종본 다시 만들어줄게.
::contentReference[oaicite:0]{index=0}
```
