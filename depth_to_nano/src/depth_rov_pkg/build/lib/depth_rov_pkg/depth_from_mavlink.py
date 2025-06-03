import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymavlink import mavutil
import argparse
import time
import threading


class DepthReaderNode(Node):
    def __init__(self, mavlink_url):
        super().__init__('depth_reader')

        self.depth_pub = self.create_publisher(Float32, '/alt_from_rov', 10)

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

        # Start thread to listen for MAVLink depth messages
        self._stop_event = threading.Event()
        self.mav_thread = threading.Thread(target=self.mavlink_loop)
        self.mav_thread.start()

    def mavlink_loop(self):
        while not self._stop_event.is_set():
            msg = self.connection.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if msg is not None:
                depth = msg.alt  # ArduSub uses alt field as depth in meters
                self.depth_pub.publish(Float32(data=depth))
                self.get_logger().info(f"Depth from ROV: {depth:.2f} m")

    def destroy_node(self):
        self._stop_event.set()
        self.mav_thread.join()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="ROS2 node to read depth from BlueROV")
    parser.add_argument('--mavlink-url', default='tcp:192.168.2.2:6777', help='MAVLink connection string')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DepthReaderNode(args.mavlink_url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
