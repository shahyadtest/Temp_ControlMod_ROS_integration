# 🔥 Temp Control Module — ROS 2 Humble Integration (UART Bridge)

This repository provides the **ROS 2 (Humble)** integration layer for the main temperature-control project:
- Main project (hardware/firmware/control): https://github.com/CrissCCL/Temp_ControlMod

It runs on a **Raspberry Pi** and bridges an embedded controller (MCU) to ROS 2 via **UART**, publishing telemetry
and receiving setpoint commands.


## 🏗️ Architecture

![Architecture](docs/architecture.svg)


## 📡 Topics

### Published
| Topic | Type | Description |
|------|------|-------------|
| `/temp` | `std_msgs/Float32` | Temperature measurement |
| `/u` | `std_msgs/Float32` | Control effort |
| `/ref` | `std_msgs/Float32` | Reference setpoint (as reported by MCU) |
| `/temp_u_ref` | `std_msgs/String` | Raw line `"temp,u,ref"` |

### Subscribed
| Topic | Type | Description |
|------|------|-------------|
| `/ref_cmd` | `std_msgs/Float32` | Setpoint command to be sent to MCU as `REF:<value>` |

### Metrics
| Topic | Type | Description |
|------|------|-------------|
| `/control_metrics` | `std_msgs/String` | Online metrics (IAE/ISE/ITAE/MAE/RMSE + overshoot/settling for steps) |



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

---

## 🧱 Build (colcon)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <THIS_REPO_URL> uart_ros_bridge_repo
cd ..
colcon build
source install/setup.bash
```

---

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

---

## 📁 Utilities

### Merge exported CSV (offline analysis)
If you export ROS bag topics to CSV files (e.g., `temp.csv`, `u.csv`, `ref.csv`), you can merge them:
```bash
python3 scripts/merge_temp_u_ref_csv.py
```

---

## 📜 License

MIT

