import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymavlink import mavutil
import argparse
import time


class DepthToMAVLinkBridge(Node):
    def __init__(self, mavlink_url):
        super().__init__('depth_to_mavlink_bridge')

        self.altitude = 0.0
        self.boot_time = time.time()  # Store boot timestamp

        # Subscribe to /depth
        self.create_subscription(Float32, '/depth', self.depth_callback, 10)

        # Setup MAVLink connection
        try:
            self.connection = mavutil.mavlink_connection(mavlink_url)
            self.connection.wait_heartbeat()
            self.connection.mav.srcSystem = 1
            self.connection.mav.srcComponent = 197
            self.get_logger().info(
                f"Connected to MAVLink system {self.connection.target_system}, component {self.connection.target_component}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            raise

        # Timer to publish to MAVLink at 5Hz
        self.create_timer(0.2, self.send_mavlink_depth)

    def depth_callback(self, msg):
        self.altitude = msg.data

    def send_mavlink_depth(self):
        # Compute time since node started (ms)
        now_ms = int((time.time() - self.boot_time) * 1000)

        # Clamp and convert depth to cm
        distance_cm = max(0, min(2500, int(-self.altitude * 100)))

        self.connection.mav.distance_sensor_send(
            time_boot_ms=now_ms,
            min_distance=0,
            max_distance=5000,
            current_distance=distance_cm,
            type=mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,
            id=0,
            orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,
            covariance=0
        )

        self.get_logger().info(f"Sent distance_sensor: {distance_cm} cm")


def main():
    parser = argparse.ArgumentParser(description="ROS2 /depth to MAVLink bridge")
    parser.add_argument('--mavlink-url', default='tcp:192.168.2.2:6777', help='MAVLink endpoint URL')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DepthToMAVLinkBridge(args.mavlink_url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
