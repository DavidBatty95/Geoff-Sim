"""LiDAR/IMU/joint-only front end; publishes estimated odometry and level clouds."""
import json
import math
import time
from array import array
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped,Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2,PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from unitree_hg.msg import LowState

from .lidar_odometry import ScanOdometry, registration_points, rotation
from .pointcloud_layout import xyz_intensity
from .sensor_geometry import matrix_to_quat_xyzw


def axis_rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == 0:
        return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    if axis == 1:
        return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))


def lidar_extrinsics(rpy, joints):
    """Robot calibration and measured joints, not scene geometry or root pose."""
    body = axis_rotation(1, rpy[1]) @ axis_rotation(0, rpy[0])
    waist = body @ axis_rotation(2, joints.get("waist_yaw_joint", 0.0))
    origin = np.array((0., 0., joints.get("floating_base_joint", 0.793)))
    origin += waist @ np.array((-0.0039635, 0., 0.035))
    waist = waist @ axis_rotation(0, joints.get("waist_roll_joint", 0.0))
    origin += waist @ np.array((0., 0., 0.019))
    waist = waist @ axis_rotation(1, joints.get("waist_pitch_joint", 0.0))
    origin += waist @ np.array((0.0576235, 0., 0.41987))
    return origin, waist @ np.diag((1., -1., -1.))


class OnboardLidarOdometry(Node):
    def __init__(self):
        super().__init__("onboard_lidar_odometry")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.estimator = ScanOdometry()
        self.imu = deque(maxlen=150)
        self.joints = deque(maxlen=150)
        self.previous_stamp = None
        self.previous_xy = np.zeros(2)
        self.velocity = np.zeros(2)
        self.last_attitude_time = -math.inf
        self.last_attitude_tick = None
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
        self.attitude_pub = self.create_publisher(Vector3Stamped, "/onboard/measured_rpy", qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/onboard/lidar", qos)
        self.level_pub = self.create_publisher(PointCloud2, "/onboard/lidar_level", qos)
        self.status_pub = self.create_publisher(String, "/onboard/localization_status", 1)
        self.tf = TransformBroadcaster(self)
        self.camera_frame = self.declare_parameter('camera_frame','mujoco_d435i_color_optical_frame').value
        self.camera_pitch = math.radians(float(self.declare_parameter('camera_mount_pitch_deg',0.).value))
        self.create_subscription(LowState, "/lowstate", self._imu, qos)
        self.create_subscription(JointState, "/joint_states", self._joints, qos)
        self.create_subscription(PointCloud2, "/livox/lidar", self._cloud, qos)

    def _imu(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        rpy,rate = tuple(msg.imu_state.rpy),float(msg.imu_state.gyroscope[2])
        if not all(map(math.isfinite,(*rpy,rate))):
            return
        self.imu.append((now,rpy,rate))
        tick = int(msg.tick)
        if tick!=self.last_attitude_tick and now-self.last_attitude_time>=.019:
            self.last_attitude_time,self.last_attitude_tick = now,tick
            attitude = Vector3Stamped()
            # LowState has a device tick, not a synchronized ROS acquisition
            # epoch. Keep the existing receipt-time contract explicit.
            attitude.header.stamp = rclpy.time.Time(seconds=now).to_msg()
            attitude.header.frame_id = 'pelvis'
            attitude.vector.x,attitude.vector.y,attitude.vector.z = map(float,rpy)
            self.attitude_pub.publish(attitude)

    def _joints(self, msg):
        self.joints.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            dict(zip(msg.name, msg.position))))

    def _cloud(self, msg):
        start = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.imu or not self.joints or (self.previous_stamp is not None and stamp <= self.previous_stamp):
            return
        imu = min(self.imu, key=lambda item: abs(item[0] - stamp))
        joints = min(self.joints, key=lambda item: abs(item[0] - stamp))
        if abs(imu[0] - stamp) > 0.15 or abs(joints[0] - stamp) > 0.15:
            return
        if max(abs(imu[1][0]), abs(imu[1][1])) > 0.65:
            return
        try:
            data = xyz_intensity(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc),throttle_duration_sec=2.)
            return
        origin, attitude = lidar_extrinsics(imu[1], joints[1])
        # Raw sensor frames are derived from measured IMU/encoders. They are
        # available to normal ROS consumers without simulator root-pose TF.
        torso = attitude@np.diag((1.,-1.,-1.))
        optical = np.array(((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.)))
        for frame,position,rotation_matrix in (
                (msg.header.frame_id,origin,attitude),
                (self.camera_frame,origin+torso@np.array((0.,.035,0.)),torso@axis_rotation(1,self.camera_pitch)@optical)):
            if frame in ('odom','base_footprint'):
                continue
            sensor_tf = TransformStamped()
            sensor_tf.header.stamp = msg.header.stamp
            sensor_tf.header.frame_id = 'base_footprint'
            sensor_tf.child_frame_id = frame
            sensor_tf.transform.translation.x,sensor_tf.transform.translation.y,sensor_tf.transform.translation.z = map(float,position)
            qx,qy,qz,qw = matrix_to_quat_xyzw(rotation_matrix)
            sensor_tf.transform.rotation.x,sensor_tf.transform.rotation.y = float(qx),float(qy)
            sensor_tf.transform.rotation.z,sensor_tf.transform.rotation.w = float(qz),float(qw)
            self.tf.sendTransform(sensor_tf)
        data[:, :3] = data[:, :3] @ attitude.T + origin
        radius = np.linalg.norm(data[:, :2], axis=1)
        floor_candidates = data[(radius > 1.0) & (radius < 8.0) & (data[:, 2] < 0.35) & (data[:, 2] > -0.35), 2]
        if len(floor_candidates) >= 30:
            ground = float(np.median(floor_candidates))
            if np.quantile(np.abs(floor_candidates - ground), 0.7) < 0.04:
                data[:, 2] -= ground
        registration,band = registration_points(data[:,:3])
        fallback = data[(radius>1.)&(radius<18.)&(data[:,2]>.25)&(data[:,2]<1.8),:2]
        accepted = self.estimator.update(registration,imu[1][2],fallback_points=fallback)
        status = String()
        status.data = json.dumps({**self.estimator.quality, "source": "lidar_imu_joint_encoders",
                                  "source_stamp": stamp, "input_age_s": self.get_clock().now().nanoseconds * 1e-9 - stamp,
                                   "cloud_points": len(data), "floor_points": len(floor_candidates),
                                   "registration_band": band,
                                  "processing_ms": (time.monotonic() - start) * 1000})
        self.status_pub.publish(status)
        self.get_logger().info(f"Registration {status.data}", throttle_duration_sec=3.0)
        if not accepted:
            return  # No fresh odometry/TF on failure: downstream freshness stops.
        xy, yaw = self.estimator.xy, self.estimator.yaw
        if self.previous_stamp is not None:
            dt = stamp - self.previous_stamp
            alpha = dt / (0.12 + dt)
            self.velocity += alpha * ((xy - self.previous_xy) / dt - self.velocity)
        self.previous_xy, self.previous_stamp = xy.copy(), stamp
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = msg.header.stamp, "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, xy)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        velocity = rotation(yaw).T @ self.velocity
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = map(float, velocity)
        odom.twist.twist.angular.z = imu[2]
        for index in (0, 7, 35):
            odom.pose.covariance[index] = 0.02
            odom.twist.covariance[index] = 0.04
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = map(float, xy)
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        msg.height,msg.width = 1,len(data)
        msg.point_step,msg.row_step = 16,16*len(data)
        msg.is_bigendian,msg.is_dense = False,True
        msg.fields = [PointField(name=name,offset=4*index,datatype=PointField.FLOAT32,count=1)
                      for index,name in enumerate(('x','y','z','intensity'))]
        msg.header.frame_id = "base_footprint"
        msg.data = array("B", data.astype("<f4").tobytes())
        self.level_pub.publish(msg)
        data[:, :2] = data[:, :2] @ rotation(yaw).T + xy
        msg.header.frame_id = "odom"
        msg.data = array("B", data.astype("<f4").tobytes())
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OnboardLidarOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
