# MAVLink ROS2 Integration Suite

This repository contains four submodules, each contributing to the integration of a BlueROV with a ROS2-based system via MAVLink. Together, they enable bidirectional communication of depth, odometry, and sensor data between the ROV and onboard or topside processing units.

---

## 🔧 Submodules Overview

### 1. `aaronmavlink/`
Created by Aaron Marburg.  
Provides minimal working examples of establishing a MAVLink connection with the BlueROV using `pymavlink`.

**Features:**
- Connects to a non-standard MAVLink endpoint (`tcpin:192.168.2.2:6777`)
- Includes two sample scripts:
  - `query_mavlink`: displays all incoming HEARTBEAT messages
  - `send_mavlink`: sends a custom HEARTBEAT with `mavlink_version = 99`

**Setup:**
1. Log into the BlueROV's web interface.
2. Enable **Pirate Mode**
3. Navigate to **MAVLink Endpoints** and manually add a new endpoint with:
   ```
   Type: TCP Server
   IP: 192.168.2.2
   Port: 6777
   ```

It’s also recommended to switch the "MAVP2P" option on the same page for improved compatibility.

**Install**:
```bash
pip install .
```

---

### 2. `depth_reader/`
Reads depth data from the BlueROV over MAVLink using `VFR_HUD.alt` and republishes it to a ROS2 topic `/alt_from_rov`.

**Command**:
```bash
python3 depth_reader.py --mavlink-url tcp:192.168.2.2:6777
```

---

### 3. `depth_to_mavlink/`
Reads depth data from a ROS2 topic `/depth` and sends it to the BlueROV via MAVLink as a `DISTANCE_SENSOR` message.

**Command**:
```bash
python3 depth_to_mavlink.py --mavlink-url tcp:192.168.2.2:6777
```

---

### 4. `ros2_mavlink_bridge/`
Full-featured bridge between ROS2 topics and MAVLink telemetry. It sends:
- `VISION_POSITION_DELTA`
- `DISTANCE_SENSOR`
- `VFR_HUD`

It also reads `GLOBAL_POSITION_INT` from MAVLink and republishes ROV depth to ROS2.

**Command**:
```bash
python3 ros2_mavlink_bridge.py --mavlink-url tcp:192.168.2.2:6777
```

---

## 🧪 Testing Workflow

1. Start `query_mavlink` from `aaronmavlink` to monitor raw MAVLink traffic.
2. Run one of the other scripts to generate and verify MAVLink messages.
3. Use `ros2 topic echo` to monitor ROS2 telemetry output.
4. Launch QGroundControl and confirm visibility of distance/odometry values.

---

## 📦 Installation Notes

Make sure all Python modules are installed per submodule. All packages support:

```bash
pip install .
```

Also ensure your ROS2 environment is sourced:
```bash
source /opt/ros/<distro>/setup.bash
```

---

## ❗ Notes

To view `DISTANCE_SENSOR` messages in QGroundControl, additional setup or tuning may be required on the ArduSub side. See internal project documentation or consult the BlueRobotics forums.

