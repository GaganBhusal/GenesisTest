import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion, Vector3, Point
from nav_msgs.msg import Odometry
import math
import time 
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ROSPublisher(Node):
    def __init__(self):
        super().__init__("go2_genesis_node")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info('ROS Publisher node started!')

    def broadcast_tf(self, pos, quat_wxyzw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])

        t.transform.rotation.x = float(quat_wxyzw[1])
        t.transform.rotation.y = float(quat_wxyzw[2])
        t.transform.rotation.z = float(quat_wxyzw[3])
        t.transform.rotation.w = float(quat_wxyzw[0])

        self.tf_broadcaster.sendTransform(t)


    def publish_imu(self, quat_wxyzw, ang_vel, lin_acc):
        msg = Imu()

        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.orientation = Quaternion(
            x = float(quat_wxyzw[1]),
            y = float(quat_wxyzw[2]),
            z = float(quat_wxyzw[3]),
            w = float(quat_wxyzw[0])
        )
        msg.angular_velocity = Vector3(
            x = float(ang_vel[0]),
            y = float(ang_vel[1]),
            z = float(ang_vel[2])
        )

        msg.linear_acceleration = Vector3(
            x = float(lin_acc[0]),
            y = float(lin_acc[1]),
            z = float(lin_acc[2])
        )

        msg.orientation_covariance[0] = -1
        msg.angular_velocity_covariance[0] = -1
        msg.linear_acceleration_covariance[0] = -1

        self.imu_publisher.publish(msg)

        self.get_logger().info('Published one imu message!')

    def publish_lidar(self, lidar_distances):
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.angle_min = -math.radians(200/2)
        msg.angle_max = math.radians(200/2)
        msg.angle_increment = math.radians(200/128)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 100.0

        msg.ranges = [float(d) for d in lidar_distances]
        msg.intensities = []
        
        self.lidar_publisher.publish(msg)

        self.get_logger().info('Published one lidar message!')

    def publish_odometry(self, pos, quat_wxyzw, lin_vel, ang_vel):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position = Point(
            x=float(pos[0]),
            y=float(pos[1]),
            z=float(pos[2])
        )

        msg.pose.pose.orientation = Quaternion(
            x=float(quat_wxyzw[1]),
            y=float(quat_wxyzw[2]),
            z=float(quat_wxyzw[3]),
            w=float(quat_wxyzw[0])
        )

        msg.twist.twist.linear = Vector3(
            x=float(lin_vel[0]),
            y=float(lin_vel[1]),
            z=float(lin_vel[2])
        )

        msg.twist.twist.angular = Vector3(
            x=float(ang_vel[0]),
            y=float(ang_vel[1]),
            z=float(ang_vel[2])
        )

        self.odometry_publisher.publish(msg)
        self.get_logger().info('Published one odometry message!')