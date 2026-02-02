# 🔥 Temp Control Module — ROS 2 Humble Integration (UART Bridge)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.x-green)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi-orange)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

## Overview

This repository provides the **ROS 2 Humble integration layer** for the main
temperature control platform:

👉 Main embedded project (hardware + firmware + control)  
https://github.com/CrissCCL/Temp_ControlMod

It runs on a **Raspberry Pi** and bridges an embedded controller (Teensy/MCU)
to ROS 2 via **UART**, enabling:

- Real-time telemetry streaming
- Remote setpoint commands
- Online control performance metrics
- Logging, visualization, and data analysis


## ✨ Features

- Bidirectional UART ↔ ROS 2 communication
- Real-time telemetry (~100 Hz, limited by UART bandwidth)
- Remote setpoint commands
- Online control metrics (IAE, ISE, ITAE, RMSE, overshoot, settling time)
- rosbag logging support
- Embedded + middleware architecture
- Designed for control engineering education and experimentation


# 📂 Contents

### ROS 2 (middleware layer)
- `/uart_ros_bridge/` → UART bridge + control metrics nodes
- `/launch/` → system launch files
- `/scripts/` → offline utilities (CSV merge, analysis)
- `/docs/` → diagrams and screenshots
- `/notebooks/` → data analysis (optional)

### Embedded firmware
- `/control_temp_LC3/` → Teensy / Arduino firmware (compiled independently, not part of colcon build)

### Package configuration
- `package.xml`
- `setup.py`


## 🏗️ Architecture

<p align="center">
<img src="https://github.com/user-attachments/assets/303aca80-6a6e-42b3-863a-67e5829025dd" alt="Architecture" width="700">
</p>


## 🔧 Hardware Setup

<p align="center">
<img src="https://github.com/user-attachments/assets/8ebc8589-6ff8-459e-b60b-cbca748937f4" width="500">
</p>

Physical connections:

- Teensy → temperature sensor + actuator (heater/fan)
- UART (Serial1) → Raspberry Pi
- Raspberry Pi → ROS 2 nodes (bridge + metrics + visualization)


# 🔌 Embedded Firmware (Teensy)

The Teensy executes the **real-time PI temperature control loop** and communicates
with ROS 2 using a simple ASCII UART protocol.

### Telemetry (MCU → ROS)

```
temp,u,ref\n
```

### Command (ROS → MCU)

```
REF:<value>\n
```

Where:

- `temp` → measured temperature
- `u` → control effort
- `ref` → active setpoint

### UART settings
- Serial1
- 57600 baud
- 8N1

Firmware is compiled with **Arduino/Teensyduino** and runs independently from ROS.


# 🎯 Use Cases

- Control systems laboratories
- Embedded–ROS integration
- Controller tuning and identification
- Digital twin experimentation
- Teaching automatic control

# 📊 Results / Visualization

### Step response (rqt_plot)

<p align="center">
<img src="https://github.com/user-attachments/assets/6322119f-1d2f-42a3-8476-69153b95efd8" width="800">
</p>

### Runtime environment (Raspberry Pi + ROS nodes)

<p align="center">
<img src="https://github.com/user-attachments/assets/b95ee6c6-966e-4d17-8ac4-cd412d26d0c2" width="800">
</p>

# 📡 ROS Topics

## Published
| Topic | Type | Description |
|------|------|-------------|
| `/temp` | Float32 | Temperature measurement |
| `/u` | Float32 | Control effort |
| `/ref` | Float32 | Active setpoint |
| `/temp_u_ref` | String | Raw telemetry line |
| `/control_metrics` | String | Online performance metrics |

## Subscribed
| Topic | Type | Description |
|------|------|-------------|
| `/ref_cmd` | Float32 | New setpoint command |



## 🔌 UART protocol

**MCU → RPi (ROS)**  
Line format (ASCII):

```
temp,u,ref\n
```

**RPi (ROS) → MCU**  
Command format (ASCII):
```
REF:<value>\n
```

Default port/baud (editable in the node file):
- Port: `/dev/serial0`
- Baud: `57600`



## ⚙️ Requirements

- Ubuntu 22.04 + ROS 2 Humble
- Python 3
- `pyserial`

Install pyserial:

```bash
pip3 install pyserial
```

> Tip (serial permissions): add your user to `dialout`
```bash
sudo usermod -a -G dialout $USER
# logout/login
```


## 🧱 Build (colcon)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <THIS_REPO_URL> uart_ros_bridge_repo
cd ..
colcon build
source install/setup.bash
```


## ▶️ Run

### Option A — Launch (recommended)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch uart_ros_bridge system.launch.py
```

### Option B — Run nodes manually
UART bridge:
```bash
ros2 run uart_ros_bridge uart_temp_to_ros
```

Metrics:
```bash
ros2 run uart_ros_bridge control_metrics
```

Send a new setpoint:
```bash
ros2 topic pub /ref_cmd std_msgs/msg/Float32 "{data: 40.0}"
```

Quick check:
```bash
ros2 topic echo /temp
ros2 topic echo /control_metrics
```

Plot:
```bash
rqt_plot /temp /ref /u
```

## 🚀 Quick Test (5-minute demo)

### 1️⃣ Launch system

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch uart_ros_bridge system.launch.py
```

### 2️⃣ Check telemetry (raw UART frame)

```bash
ros2 topic echo /temp_u_ref
```

### 3️⃣ Check individual signals

```bash
ros2 topic echo /temp
ros2 topic echo /u
ros2 topic echo /ref
```

### 4️⃣ Change setpoint (one-shot)

```bash
ros2 topic pub --once /ref_cmd std_msgs/msg/Float32 "{data: 40.0}"
```
### 5️⃣ Visualize response

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rqt
```

You should observe:

- temperature tracking the reference
- control effort reacting to the error
- stable closed-loop behavior

## 📁 Utilities

### Merge exported CSV (offline analysis)
If you export ROS bag topics to CSV files (e.g., `temp.csv`, `u.csv`, `ref.csv`), you can merge them:
```bash
python3 scripts/merge_temp_u_ref_csv.py
```

### 🔗 Related repositories 

- **Embedded firmware (MCU / control loop)**
- 👉 https://github.com/CrissCCL/Temp_ControlMod

## 🤝 Support projects

 Support me on Patreon [https://www.patreon.com/c/CrissCCL](https://www.patreon.com/c/CrissCCL)

## 📜 License

MIT License  

