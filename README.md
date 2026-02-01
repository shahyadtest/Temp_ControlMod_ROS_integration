# 🔥 Temp Control Module — ROS 2 Humble Integration (UART Bridge)

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


# 📂 Contents

### ROS 2 (middleware layer)
- `/uart_ros_bridge/` → UART bridge + control metrics nodes
- `/launch/` → system launch files
- `/scripts/` → offline utilities (CSV merge, analysis)
- `/docs/` → diagrams and screenshots
- `/notebooks/` → data analysis (optional)

### Embedded firmware
- `/control_temp_LC/` → Teensy / Arduino control firmware (C/C++)

### Package configuration
- `package.xml`
- `setup.py`


## 🏗️ Architecture

<p align="center">
<img src="https://github.com/user-attachments/assets/303aca80-6a6e-42b3-863a-67e5829025dd" alt="Architecture" width="700">
</p>

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
ros2 launch uart_ros_bridge temp_control.launch.py
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
ros2 topic pub /ref_cmd std_msgs/Float32 "{data: 60.0}"
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

