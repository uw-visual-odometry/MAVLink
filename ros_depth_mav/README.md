# ROS2 `/depth` to MAVLink Bridge Node  
This ROS 2 node listens to a ROS2 topic (`/depth`) that publishes depth in meters and republishes that data to a MAVLink endpoint as a `DISTANCE_SENSOR` message. This is useful for sending ROV-reported depth data to a companion autopilot or topside system using MAVLink.

## Overview  
This script:  
* Subscribes to a ROS2 topic `/depth` containing depth data in meters.  
* Connects to a MAVLink telemetry stream (e.g., over TCP or serial).  
* Waits for a heartbeat to ensure the connection is alive.  
* Periodically sends a `DISTANCE_SENSOR` MAVLink message using the received depth.  
* Converts the depth to centimeters (negated and clamped to MAVLink range).  

This node is useful for transmitting depth measurements to systems expecting MAVLink sensor input (e.g., ArduSub or QGroundControl).

## How it works

### ROS2 Node: DepthToMAVLinkBridge  
* Subscriber:  
  * Topic: `/depth`  
  * Message Type: `std_msgs/msg/Float32`  
  * Receives: Depth in meters from another ROS2 node  
* MAVLink Connection:  
  * Uses `pymavlink` to connect to the vehicle via a URL (e.g., `tcp:192.168.2.2:6777`).  
  * Waits for a MAVLink heartbeat to verify the connection.  
  * Initializes with system ID 1 and component ID 197.  
* Distance Sensor Message:  
  * Type: `DISTANCE_SENSOR`  
  * Rate: 5 Hz (via ROS2 timer)  
  * Fields:
    * `current_distance`: calculated as `-depth * 100` (clamped to 0–2500 cm)  
    * `orientation`: `MAV_SENSOR_ROTATION_NONE`  
    * `type`: `MAV_DISTANCE_SENSOR_UNKNOWN`  

## Usage  
python3 depth_to_mavlink.py --mavlink-url tcp:192.168.2.2:6777  
(Replace the IP with your actual ROV's IP)

## Dependencies  
* rclpy (ROS2 Python client library)  
* pymavlink :: pip install pymavlink (if needed)  
* std_msgs from ROS2 interfaces

## Installation  
* Use pip install . to build the executable

## Tips  
* If you're using ROS 2, make sure  
  source /opt/ros/<distro>/setup.bash  
  is done first.
