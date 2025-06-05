# ROS2 to MAVLink Bridge Node

This ROS2 node bridges key telemetry from ROS2 topics (`/Odometry/orbSlamOdom`, `/imu`, and `/Depth`) to MAVLink messages such as `vision_position_delta`, `distance_sensor`, and `vfr_hud`. It also listens for incoming MAVLink messages (like `GLOBAL_POSITION_INT`) and republishes ROV depth as a ROS2 topic (`/rangefinder_distance`).

## Overview

This script:
* Subscribes to ROS2 topics for odometry, IMU, and range sensor data.
* Computes position and angle deltas.
* Sends relevant MAVLink telemetry:
  * `VISION_POSITION_DELTA`
  * `DISTANCE_SENSOR`
  * `VFR_HUD`
* Publishes the depth from MAVLink `GLOBAL_POSITION_INT` to a ROS2 topic.

## How it works

### ROS2 Node: ROS2MAVLinkBridge

* **Subscriptions**:
  * `/Odometry/orbSlamOdom` → `nav_msgs/msg/Odometry` → velocity + orientation
  * `/Depth` → `sensor_msgs/msg/Range` → ROV-reported depth
  * `/imu` → `sensor_msgs/msg/Imu` → orientation for fallback

* **MAVLink Connection**:
  * Uses `pymavlink` to connect to the vehicle (default: `tcp:192.168.2.2:6777`)
  * Sets MAVLink `srcSystem = 1`, `srcComponent = 197`

* **Outgoing MAVLink Messages**:
  * `VISION_POSITION_DELTA`  
    - Sent at 10 Hz  
    - Computes delta in ENU frame, inverts Z  
    - Uses odometry or integrates IMU + velocity if odometry is stale  
  * `DISTANCE_SENSOR`  
    - Sent at 10 Hz  
    - Converts range data to centimeters  
  * `VFR_HUD`  
    - Publishes altitude (positive) and velocity norm  
    - Inverts Z velocity for NED frame

* **Incoming MAVLink Handling**:
  * Listens for `GLOBAL_POSITION_INT`  
    - Extracts `relative_alt` in mm and publishes as `/rangefinder_distance`

## Usage

```bash
python3 ros2_mavlink_bridge.py --mavlink-url tcp:192.168.2.2:6777
```

> Replace the IP with your MAVLink endpoint.

## Dependencies

* rclpy (ROS2 Python client library)
* pymavlink → install with `pip install pymavlink`
* scipy → install with `pip install scipy`
* ROS2 message types:
  - `nav_msgs/msg/Odometry`
  - `std_msgs/msg/Float32`
  - `sensor_msgs/msg/Imu`, `Range`

## Tips

* Source your ROS 2 environment first:

```bash
source /opt/ros/<distro>/setup.bash
```

* Ensure all relevant ROS2 topics (`/Odometry/orbSlamOdom`, `/Depth`, `/imu`) are active.
* The MAVLink autopilot must send heartbeats and accept messages on port `6777` or the one specified.

