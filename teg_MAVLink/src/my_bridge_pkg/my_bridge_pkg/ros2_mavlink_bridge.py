# rclpy gives us the ability to create ROS2 nodes
# Node is the base class for all ROS2 nodes
# Odometry is a ROS2 message type that contains information about the position and velocity of a robot
# Float32 is a ROS2 message type that contains a single float value
# Imu is a ROS2 message type that contains information about the orientation and angular velocity of a robot
# mavutil is a pymavlink module that provides a connection to a MAVLink system
# euler_from_quaternion is a function that converts a quaternion to Euler angles
# argparse is a module for parsing command-line arguments
# time is a module for working with time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from pymavlink import mavutil
from tf_transformations import euler_from_quaternion
import argparse
import time


class ROS2MAVLinkBridge(Node):
    def __init__(self, mavlink_url):
        super().__init__('ros2_mavlink_bridge')

        self.velocity = (0.0, 0.0, 0.0) 
        self.altitude = 0.0
        self.last_time_us = time.time_ns() // 1000

        self.angle_delta_odom = [0.0, 0.0, 0.0]
        self.angle_delta_imu = [0.0, 0.0, 0.0]
        self.prev_odom_angles = None
        self.prev_imu_angles = None
        self.prev_odom_time_us = None
        self.prev_imu_time_us = None

        self.position_delta = [0.0, 0.0, 0.0]
        self.prev_odom_position = None
        self.odom_timeout_us = 500_000  # 0.5 sec

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/depth', self.altitude_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.rangefinder_pub = self.create_publisher(Float32, '/rangefinder_distance', 10)

        try:
            self.connection = mavutil.mavlink_connection(mavlink_url)
            self.connection.wait_heartbeat()
            self.connection.mav.srcSystem = 1
            self.connection.mav.srcComponent = 197
            self.get_logger().info(f"Connected to MAVLink system {self.connection.target_system}, component {self.connection.target_component}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            raise

        self.create_timer(0.1, self.send_mavlink)               # 10 Hz
        self.create_timer(0.2, self.read_mavlink_rangefinder)   # 5 Hz

    def odom_callback(self, msg):
        now_us = time.time_ns() // 1000
        self.prev_odom_time_us = now_us

        self.velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        )

        pos = msg.pose.pose.position
        current_pos = (pos.x, pos.y, pos.z)
        if self.prev_odom_position is not None:
            self.position_delta = [
                current_pos[i] - self.prev_odom_position[i] for i in range(3)
            ]
        self.prev_odom_position = current_pos

        q = msg.pose.pose.orientation
        angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if self.prev_odom_angles is not None:
            self.angle_delta_odom = [
                angles[i] - self.prev_odom_angles[i] for i in range(3)
            ]
        self.prev_odom_angles = angles

    def altitude_callback(self, msg):
        self.altitude = msg.data

    def imu_callback(self, msg):
        now_us = time.time_ns() // 1000
        self.prev_imu_time_us = now_us

        q = msg.orientation
        try:
            angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
            if self.prev_imu_angles:
                self.angle_delta_imu = [
                    angles[i] - self.prev_imu_angles[i] for i in range(3)
                ]
            self.prev_imu_angles = angles
        except Exception as e:
            self.get_logger().warn(f"IMU conversion failed: {e}")

    def send_mavlink(self):
        now_us = time.time_ns() // 1000
        now_ms = now_us // 1000
        dt_us = now_us - self.last_time_us
        self.last_time_us = now_us
        dt_sec = dt_us / 1e6

        vx, vy, vz = self.velocity
        fallback_position_delta = [vx * dt_sec, vy * dt_sec, vz * dt_sec]

        if self.prev_odom_time_us and (now_us - self.prev_odom_time_us) < self.odom_timeout_us:
            pos_delta_enu = self.position_delta
            angle_delta = self.angle_delta_odom
        else:
            pos_delta_enu = fallback_position_delta
            angle_delta = self.angle_delta_imu
            self.get_logger().warn("Odom timeout: using IMU angle delta and velocity integration")

        position_delta = [
            pos_delta_enu[0],
            pos_delta_enu[1],
            -pos_delta_enu[2]
        ]

        self.connection.mav.vision_position_delta_send(
            time_usec=now_us,
            time_delta_usec=dt_us,
            angle_delta=angle_delta,
            position_delta=position_delta,
            confidence=100.0
        )

        distance_cm = max(0, min(2500, int(-self.altitude * 100)))
        self.connection.mav.distance_sensor_send(
            now_ms,
            0, 5000,
            distance_cm,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,
            0,
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            0
        )

        vx_ned, vy_ned, vz_ned = vx, vy, -vz
        groundspeed = (vx_ned**2 + vy_ned**2 + vz_ned**2) ** 0.5
        self.connection.mav.vfr_hud_send(
            airspeed=0.0,
            groundspeed=groundspeed,
            heading=0,
            throttle=0,
            alt=self.altitude,
            climb=vz_ned
        )

    def read_mavlink_rangefinder(self):
        msg = self.connection.recv_match(type='VFR_HUD', blocking=False)
        if msg and hasattr(msg, 'alt'):
            altitude = float(msg.alt)
            rangefinder = Float32()
            rangefinder.data = -altitude
            self.rangefinder_pub.publish(rangefinder)
            self.get_logger().info(f"Rangefinder distance from MAVLink: {-altitude:.2f} m")


def main():
    parser = argparse.ArgumentParser(description="ROS2 to MAVLink Bridge")
    parser.add_argument('--mavlink-url', default='tcp:192.168.2.2:6777', help='MAVLink endpoint URL')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ROS2MAVLinkBridge(args.mavlink_url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
