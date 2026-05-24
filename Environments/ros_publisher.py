import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
from geometry_msgs.msg import Quaternion, Vector3, Point
from nav_msgs.msg import Odometry
import math
import time 
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from slam_toolbox.srv import Reset
from geometry_msgs.msg import Twist

class ROSPublisher(Node):
    def __init__(self):
        super().__init__("go2_genesis_node")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.lidar_publisher_2d = self.create_publisher(LaserScan, '/scan', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.lidar_3d_publisher = self.create_publisher(PointCloud2, '/scan_3d', 10)


        self.latest_vx = 0.0
        self.latest_wz = 0.0
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        self.get_logger().info('ROS Publisher node started!')

    def _cmd_vel_callback(self, msg):
        self.latest_vx = msg.linear.x
        self.latest_wz = msg.angular.z


    def publish_all(self, pos, quat_wxyzw, lin_vel, ang_vel, lin_acc, lidar_points):

        current_time = self.get_clock().now().to_msg()
        # Processing Data Here

        pos = pos.squeeze(0)
        quat_wxyzw = quat_wxyzw.squeeze(0)
        lin_vel = lin_vel.squeeze(0)
        ang_vel = ang_vel.squeeze(0)    
        lin_acc = lin_acc.squeeze(0)

        """    
        For lidar distances we do 
        lidar_distances = lidar_points[1].squeeze(0)[:, 32]
        32 Means we take the 32nd point from the 64 points, which is the front straight point of the robot.
        """
        lidar_distances = lidar_points[1].squeeze(0)[:, 30:35]
        lidar_points = lidar_points[0].squeeze(0)

        self.broadcast_tf(pos, quat_wxyzw, current_time)
        self.publish_imu(quat_wxyzw, ang_vel, lin_acc, current_time)
        self.publish_odometry(pos, quat_wxyzw, lin_vel, ang_vel, current_time)

        self.publish_lidar_2d(lidar_distances, current_time)
        self.publish_lidar_3d(lidar_points, current_time)


    def broadcast_tf(self, pos, quat_wxyzw, current_time):
        t = TransformStamped()
        t.header.stamp = current_time
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


    def publish_imu(self, quat_wxyzw, ang_vel, lin_acc, current_time):
        msg = Imu()

        msg.header.stamp    = current_time
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

        # self.get_logger().info('Published one imu message!')

    def publish_lidar_2d(self, lidar_distances, current_time):
        
        lidar_distances = np.min(lidar_distances.detach().cpu().numpy().astype(np.float32), axis=1)
        msg = LaserScan()

        msg.header.stamp = current_time
        msg.header.frame_id = 'base_link'

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi/128
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.5
        msg.range_max = 8.0

        msg.ranges = [float(d) for d in lidar_distances]
        msg.intensities = []
        
        self.lidar_publisher_2d.publish(msg)

        # self.get_logger().info('Published one lidar message!')


    def publish_lidar_3d(self, lidar_points, current_time):
        points_np = lidar_points.detach().cpu().numpy().astype(np.float32)
        points_np = points_np.reshape(-1, 3)
        valid = (
            np.isfinite(points_np).all(axis=1) &
            (np.linalg.norm(points_np, axis=1) > 0.05)
        )
        points_np = points_np[valid]

        height_mask = (points_np[:, 2] > -0.3) & (points_np[:, 2] < 1)
        points_np = points_np[height_mask]

        distance_mask = np.linalg.norm(points_np[:, :2], axis=1) > 0.5
        points_np = points_np[distance_mask]

        range_mask = np.linalg.norm(points_np, axis=1) < 8.0
        points_np = points_np[range_mask]

        # def voxel_downsample(points, voxel_size=0.1):
        #     voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        #     _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        #     return points[unique_idx]

        # points_np = voxel_downsample(points_np, voxel_size=0.1)

        msg = PointCloud2()
        msg.header.stamp = current_time
        msg.header.frame_id = 'base_link'
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        msg.height = 1
        msg.width = points_np.shape[0]
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        
        msg.data = points_np.tobytes()

        self.lidar_3d_publisher.publish(msg)
        # self.get_logger().info('Published 3D lidar message!')



    def publish_odometry(self, pos, quat_wxyzw, lin_vel, ang_vel, current_time):
        msg = Odometry()
        msg.header.stamp = current_time
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
        # self.get_logger().info('Published one odometry message!'aru p