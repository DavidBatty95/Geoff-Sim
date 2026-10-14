import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from unitree_social_navigation_msgs.msg import TrackedPerson, TrackedPersons
from tf2_ros import Buffer, TransformException, TransformListener

from .pointcloud import iter_xyz


class ScanDynamicFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_dynamic_filter")
        self.declare_parameter("input_scan_topic", "/scan_raw")
        self.declare_parameter("output_scan_topic", "/scan")
        self.declare_parameter("tracked_cloud_topic", "/tracked_dynamic_obstacles")
        self.declare_parameter("filter_radius", 0.75)
        self.declare_parameter("range_padding", 0.45)
        self.declare_parameter("cloud_timeout", 0.8)
        self.declare_parameter("min_filter_range", 0.20)

        self.filter_radius = float(self.get_parameter("filter_radius").value)
        self.range_padding = float(self.get_parameter("range_padding").value)
        self.cloud_timeout = float(self.get_parameter("cloud_timeout").value)
        self.min_filter_range = float(self.get_parameter("min_filter_range").value)
        if self.filter_radius <= 0.0:
            raise ValueError("filter_radius must be positive")
        if self.range_padding < 0.0:
            raise ValueError("range_padding must be non-negative")
        if self.cloud_timeout <= 0.0:
            raise ValueError("cloud_timeout must be positive")
        if self.min_filter_range < 0.0:
            raise ValueError("min_filter_range must be non-negative")
        self.latest_cloud: PointCloud2 | None = None
        self.latest_people = None
        self.carrying = False
        self.payload = None
        self.pending_scan = None
        self.last_scan_stamp = -math.inf
        self.latest_cloud_time = self.get_clock().now() - Duration(seconds=999.0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(LaserScan, str(self.get_parameter("output_scan_topic").value), 10)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("tracked_cloud_topic").value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(TrackedPersons, "/tracked_persons", self._people_callback, 1)
        self.create_subscription(Bool,'/navigation/carrying',lambda msg:setattr(self,'carrying',bool(msg.data)),1)
        self.create_subscription(PointStamped,'/navigation/payload_pose',lambda msg:setattr(self,'payload',msg),1)
        self.create_timer(.01,self._retry_scan)
        self.get_logger().info("Filtering confirmed dynamic people out of /scan for SLAM")

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.latest_cloud_time = self.get_clock().now()

    def _people_callback(self, msg: TrackedPersons) -> None:
        self.latest_people = msg

    def _scan_callback(self, msg: LaserScan) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds*1e-9
        if stamp<=self.last_scan_stamp:
            return
        if self.pending_scan is not None:
            previous = rclpy.time.Time.from_msg(self.pending_scan[0].header.stamp).nanoseconds*1e-9
            if stamp<=previous:
                return
        started = time.monotonic() if self.pending_scan is None else self.pending_scan[1]
        self.pending_scan = msg,started
        self._retry_scan()

    def _publish_scan(self,filtered):
        stamp = rclpy.time.Time.from_msg(filtered.header.stamp).nanoseconds*1e-9
        if stamp>self.last_scan_stamp:
            self.pub.publish(filtered)
            self.last_scan_stamp = stamp
        self.pending_scan = None

    def _retry_scan(self):
        if self.pending_scan is not None:
            self._process_scan(self.pending_scan[0],time.monotonic()-self.pending_scan[1]<.2)

    def _process_scan(self,msg,defer_transform=True):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = list(msg.intensities)
        filtered.ranges = list(msg.ranges)
        if self.carrying and self.payload is not None and self.payload.header.frame_id == msg.header.frame_id:
            age = (self.get_clock().now()-rclpy.time.Time.from_msg(self.payload.header.stamp)).nanoseconds*1e-9
            if 0. <= age <= .25:
                self._remove_dynamic_return(filtered,self.payload.point.x,self.payload.point.y,.14)

        source = self.latest_people if self.latest_people is not None else self.latest_cloud
        if source is None or (source is self.latest_people and not source.persons):
            self._publish_scan(filtered)
            return
        cloud_age = (self.get_clock().now() - rclpy.time.Time.from_msg(source.header.stamp)).nanoseconds * 1e-9
        if not 0.0 <= cloud_age <= self.cloud_timeout:
            self._publish_scan(filtered)
            return

        transform = None
        if source.header.frame_id and source.header.frame_id != msg.header.frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(msg.header.frame_id, source.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            except TransformException as exc:
                if defer_transform:
                    # TF and cloud packets use different subscriptions. Release
                    # the executor so the acquisition-time transform can arrive;
                    # retain only the latest scan and never restamp it as fresh.
                    return
                self.get_logger().warn(
                    f"No transform {msg.header.frame_id} <- {source.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0,
                )
                self._publish_scan(filtered)
                return

        removed = 0
        if self.latest_people is not None:
            offset = (rclpy.time.Time.from_msg(msg.header.stamp) - rclpy.time.Time.from_msg(source.header.stamp)).nanoseconds * 1e-9
            people = [(p.pose.pose.position.x + p.velocity.twist.linear.x * offset,
                       p.pose.pose.position.y + p.velocity.twist.linear.y * offset, 0.0, p.body_radius + 0.10)
                      for p in source.persons if p.confidence >= 0.5 and p.track_state == TrackedPerson.TRACK_STATE_CONFIRMED]
        else:
            people = [(x, y, z, None) for x, y, z in iter_xyz(source)]
        for x, y, z, radius in people:
            if transform is not None:
                x, y, z = self._transform_point(transform, x, y, z)
            removed += self._remove_dynamic_return(filtered, x, y, radius)
        if removed:
            self.get_logger().info(f"Removed {removed} scan bins near confirmed dynamic people", throttle_duration_sec=2.0)
        self._publish_scan(filtered)

    def _remove_dynamic_return(self, scan: LaserScan, x: float, y: float, body_radius=None) -> int:
        distance = math.hypot(x, y)
        if distance < max(scan.range_min, self.min_filter_range) or distance > scan.range_max:
            return 0
        angle = math.atan2(y, x)
        if angle < scan.angle_min or angle > scan.angle_max:
            return 0
        center = int(round((angle - scan.angle_min) / scan.angle_increment))
        radius = self.filter_radius if body_radius is None else body_radius
        half_width = max(1, int(math.ceil(math.atan2(radius, max(distance, 1e-3)) / scan.angle_increment)))
        removed = 0
        for index in range(max(0, center - half_width), min(len(scan.ranges), center + half_width + 1)):
            value = scan.ranges[index]
            if not math.isfinite(value):
                continue
            if body_radius is not None:
                ray_angle = scan.angle_min + index * scan.angle_increment
                if math.hypot(value * math.cos(ray_angle) - x, value * math.sin(ray_angle) - y) <= body_radius:
                    scan.ranges[index] = math.inf
                    removed += 1
                continue
            # Remove returns at the person range and any immediate background that would smear into the map.
            if value >= distance - self.filter_radius and value <= distance + self.filter_radius + self.range_padding:
                scan.ranges[index] = math.inf
                removed += 1
        return removed

    @staticmethod
    def _transform_point(transform, x: float, y: float, z: float) -> tuple[float, float, float]:
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        uvx = q.y * z - q.z * y
        uvy = q.z * x - q.x * z
        uvz = q.x * y - q.y * x
        uuvx = q.y * uvz - q.z * uvy
        uuvy = q.z * uvx - q.x * uvz
        uuvz = q.x * uvy - q.y * uvx
        rx = x + 2.0 * (q.w * uvx + uuvx)
        ry = y + 2.0 * (q.w * uvy + uuvy)
        rz = z + 2.0 * (q.w * uvz + uuvz)
        return rx + tx, ry + ty, rz + tz


def main() -> None:
    rclpy.init()
    node = ScanDynamicFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
