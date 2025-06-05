# MAVLink to ROS2 Depth Reader Node
This ROS 2 node listens to a MAVLink connection (typically from a BlueROV running ArduSub) and republishes the depth (altitude) data to a ROS2 topic. It bridges the MAVLink VFR_HUD message's alt field, which ArduSub uses to represent depth in meters.

## Overview
This script:
* Connects to a MAVLink telemetry stream (e.g., over TCP or serial).
* Waits for a heartbeat to ensure the connection is alive.
* Continuously listens for VFR_HUD MAVLink messages.
* Extracts the alt (altitude/depth) field from each message.
* Publishes this depth value as a Float32 ROS2 message to the topic /alt_from_rov.
This node is useful for systems that need ROS2 access to live depth telemetry from an ROV (Remotely Operated Vehicle).

## How it works

### ROS2 Node: DepthReaderNode
* Publisher:
  * Topic: /alt_from_rov
  * Message Type: std_msgs/msg/Float32
  * Publishes: The depth in meters from the ROV, derived from the VFR_HUD.alt field.
* MAVLink Connection:
  * Uses pymavlink to connect to the vehicle via a URL (e.g., tcp:192.168.2.2:6777).
  * Waits for a MAVLink heartbeat to verify the connection.
  * Starts a background thread (mavlink_loop) that receives messages.
* Depth Extraction:
  * MAVLink message type: VFR_HUD
  * Extracts the .alt field (depth in meters) and publishes it.

## Usage

python3 depth_reader.py --mavlink-url tcp:192.168.2.2:6777 (replace ip with your ROVs ip)

## Dependencies
* rclpy (ROS2 Python client library)
* pymavlink ::pip install pymavlink (if needed)
* std_msgs from ROS2 interfaces

## Installation
* Use pip install . to build the executable

## Tips
* If you're using ROS 2, make sure source /opt/ros/<distro>/setup.bash is done first.
