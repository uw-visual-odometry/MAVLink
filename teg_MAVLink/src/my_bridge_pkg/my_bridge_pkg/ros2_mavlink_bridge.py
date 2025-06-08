# rclpy gives us the ability to create ROS2 nodes
# Node is the base class for all ROS2 nodes
# Odometry is a ROS2 message type that contains information about the position and velocity of a robot
# Float32 is a ROS2 message type that contains a single float value
# Imu is a ROS2 message type that contains information about the orientation and angular velocity of a robot
# mavutil is a pymavlink module that provides a connection to a MAVLink system
# R.from_quat is a scipy function that converts a quaternion to Euler angles
# argparse is a module for parsing command-line arguments
# time is a module for working with time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R
import argparse
import time


class ROS2MAVLinkBridge(Node):
    def __init__(self, mavlink_url):
        super().__init__('ros2_mavlink_bridge')

        # initialize velocity, distance, and time
        self.velocity = (0.0, 0.0, 0.0)
        self.altitude = 0.0
        self.last_time_us = time.time_ns() // 1000

        self.angle_delta_odom = [0.0, 0.0, 0.0]
        self.angle_delta_imu = [0.0, 0.0, 0.0]
        self.prev_odom_angles = None
        self.prev_imu_angles = None
        self.prev_odom_time_us = None
        self.prev_imu_time_us = None
        self.last_velocity_time_us = 0

        self.position_delta = [0.0, 0.0, 0.0]
        self.prev_odom_position = None

        self.odom_timeout_us = 500_000  # 0.5 sec
        self.altitude_timeout_us = 500_000  # 0.5 sec
        self.last_altitude_time_us = 0

        # Create subscriptions to Odometry pose topic, depth (altitude of ROV), and Jetson Nano IMU
        self.create_subscription(Odometry, '/Odometry/orbSlamOdom', self.odom_callback, 10)
        self.create_subscription(Range, '/Depth', self.altitude_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # Create publisher for the depth from the ROV BAR30 sensor
        self.rangefinder_pub = self.create_publisher(Float32, '/rangefinder_distance', 10)

        # Initialize time
        self.start_time_monotonic = time.monotonic()
        self.last_time_us = 0
        # try and except the connection via mavlink from nano to pi
        try:
            self.connection = mavutil.mavlink_connection(mavlink_url)
            self.connection.wait_heartbeat()
            self.connection.mav.srcSystem = 1
            self.connection.mav.srcComponent = 197  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
            self.get_logger().info(f"Connected to MAVLink system {self.connection.target_system}, component {self.connection.target_component}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            raise

        # The mavlink from nano to pi should be sent at 10 Hz
        self.create_timer(0.1, self.send_mavlink)               # 10 Hz
        # Read the pi depth from 5 Hz
        self.create_timer(0.2, self.read_mavlink_rangefinder)   # 5 Hz

    # retrieve the odometry topic fields, such as velocity, pose, and orientation
    def odom_callback(self, msg):
        now_us = time.time_ns() // 1000
        self.prev_odom_time_us = now_us
        self.last_velocity_time_us = now_us

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
        angles = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        if self.prev_odom_angles is not None:
            self.angle_delta_odom = [
                angles[i] - self.prev_odom_angles[i] for i in range(3)
            ]
        self.prev_odom_angles = angles

    # retrieve the altitude of the ROV from the range message
    def altitude_callback(self, msg):
        self.get_logger().info(f"Full Range message: {msg}")
        self.altitude = msg.range
        self.last_altitude_time_us = time.time_ns() // 1000

        alt_m = self.altitude
        print(f"this is the alt_m: {alt_m:.3f} m")  # No more curly braces!

    # retrive the imu sensor orientation 
    def imu_callback(self, msg):
        now_us = time.time_ns() // 1000
        self.prev_imu_time_us = now_us

        q = msg.orientation
        try:
            angles = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            if self.prev_imu_angles is not None:
                self.angle_delta_imu = [
                    angles[i] - self.prev_imu_angles[i] for i in range(3)
                ]
            else:
                self.angle_delta_imu = [0.0, 0.0, 0.0]

            self.prev_imu_angles = angles
        except Exception as e:
            self.get_logger().warn(f"IMU conversion failed: {e}")

    # send the mavlink information
    def send_mavlink(self):
        now_monotonic = time.monotonic()
        now_us = int((now_monotonic - self.start_time_monotonic) * 1e6)
        now_ms = now_us // 1000
        dt_us = now_us - (self.last_time_us or 0)
        self.last_time_us = now_us
        dt_sec = dt_us / 1e6

        if self.prev_odom_time_us and (now_us - self.prev_odom_time_us) < self.odom_timeout_us:
            pos_delta_enu = self.position_delta
            angle_delta = self.angle_delta_odom
        else:
            if (now_us - self.last_velocity_time_us) > self.odom_timeout_us:
                vx, vy, vz = (0.0, 0.0, 0.0)
            else:
                vx, vy, vz = self.velocity
            pos_delta_enu = [vx * dt_sec, vy * dt_sec, vz * dt_sec]
            angle_delta = self.angle_delta_imu
            self.get_logger().warn("Odom timeout: using IMU angle delta and velocity integration")

        # transfer from enu to frd, swap e and n values and flip the up to down
        position_delta = [
            pos_delta_enu[1],
            pos_delta_enu[0],
            -pos_delta_enu[2]
        ]

        alt_m = self.altitude
        print("this is the alt_m", {alt_m})
        distance_cm = max(0, min(2500, int(alt_m * 100)))

        # vision position delta message format
        # confidence is used to tell the flight controller confidence of values
        # if set to a dependent value, it would be better for controls integration
        self.connection.mav.vision_position_delta_send(
            time_usec=now_us,
            time_delta_usec=dt_us,
            angle_delta=angle_delta,
            position_delta=position_delta,
            confidence=100.0
        )

        # send the distance / altitude values to the Pi
        # can variably send the covariance to higher values if odometry breaks
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

        # send groundspeed (sqrt of x, y, and z components squared) to the hud ** provides no functionality to flight controller
        vx_ned, vy_ned, vz_ned = self.velocity[0], self.velocity[1], -self.velocity[2]
        groundspeed = (vx_ned**2 + vy_ned**2 + vz_ned**2) ** 0.5
        self.connection.mav.vfr_hud_send(
            airspeed=0.0,
            groundspeed=groundspeed,
            heading=0,
            throttle=0,
            alt=alt_m,
            climb=vz_ned
        )
        self.get_logger().info(f"groundspeed: {groundspeed:.2f} m/s")
        self.get_logger().info(f"distance: {distance_cm:.2f} cm")
        self.get_logger().info(f"time delta: {dt_us:.2f} us, position delta {position_delta}, angle delta {angle_delta}")



    def read_mavlink_rangefinder(self): 
        msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1) 
        if msg is not None:
            # relative_alt is in millimeters above home position
            depth = -float(msg.relative_alt) / 1000.0  # Convert mm to meters and invert for depth
            self.rangefinder_pub.publish(Float32(data=depth))
            self.get_logger().info(f"Depth from ROV (GLOBAL_POSITION_INT): {depth:.2f} m")
        else:
            self.get_logger().warn("No GLOBAL_POSITION_INT message received")


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
