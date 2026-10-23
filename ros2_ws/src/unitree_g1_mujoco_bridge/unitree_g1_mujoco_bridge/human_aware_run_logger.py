import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist,TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String,Float64MultiArray,Bool


class HumanAwareRunLogger(Node):
    def __init__(self) -> None:
        super().__init__("human_aware_run_logger")
        self.declare_parameter("output_path", "")
        self.declare_parameter("flush_every", 1)
        self.declare_parameter('record_control_state',False)
        self.declare_parameter("planner_debug_topic", "/human_avoidance_debug")
        self.declare_parameter("safety_debug_topic", "/human_collision_avoidance_debug")
        self.declare_parameter("tracker_debug_topic", "/dynamic_obstacle_tracker/debug_tracks")
        self.declare_parameter("state_topic", "/human_avoidance_state")
        self.declare_parameter("raw_cmd_topic", "/cmd_vel")
        self.declare_parameter("planner_cmd_topic", "/cmd_vel_human_aware")
        self.declare_parameter("safe_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("people_topic", "/mujoco/pedestrians")
        self.declare_parameter("benchmark_metrics_topic", "/benchmark/metrics")

        output_path = str(self.get_parameter("output_path").value).strip()
        if not output_path:
            output_path = "/tmp/human_aware_run.jsonl"
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.output_path.open("a", buffering=1)
        self.flush_every = max(1, int(self.get_parameter("flush_every").value))
        self.write_count = 0

        self.create_subscription(String, str(self.get_parameter("planner_debug_topic").value), lambda msg: self._string_callback("planner_debug", msg), 10)
        self.create_subscription(String, str(self.get_parameter("safety_debug_topic").value), lambda msg: self._string_callback("safety_debug", msg), 10)
        self.create_subscription(String, str(self.get_parameter("tracker_debug_topic").value), lambda msg: self._string_callback("tracker_debug", msg), 10)
        self.create_subscription(String, str(self.get_parameter("state_topic").value), lambda msg: self._string_callback("planner_state", msg), 10)
        self.create_subscription(String, str(self.get_parameter("people_topic").value), lambda msg: self._string_callback("mujoco_people", msg), 10)
        self.create_subscription(String, str(self.get_parameter("benchmark_metrics_topic").value), lambda msg: self._string_callback("benchmark_metrics", msg), 10)
        self.create_subscription(Twist, str(self.get_parameter("raw_cmd_topic").value), lambda msg: self._twist_callback("raw_cmd", msg), 10)
        self.create_subscription(Twist, str(self.get_parameter("planner_cmd_topic").value), lambda msg: self._twist_callback("planner_cmd", msg), 10)
        self.create_subscription(Twist, str(self.get_parameter("safe_cmd_topic").value), lambda msg: self._twist_callback("safe_cmd", msg), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_callback, 10)
        self.create_subscription(Odometry, "/evaluation/odom", lambda msg: self._odom_callback(msg, "evaluation_odom"), 10)
        self.create_subscription(String,"/evaluation/aligned_state",lambda msg:self._string_callback('evaluation_aligned',msg),10)
        self.create_subscription(String, "/onboard/localization_status", lambda msg: self._string_callback("localization_status", msg), 10)
        self.create_subscription(String, "/navigation/replanning_status", lambda msg: self._string_callback("replanning_status", msg), 10)
        self.create_subscription(String,'/g1/policy_timing',lambda msg:self._string_callback('policy_timing',msg),100)
        self.create_subscription(String,'/g1/lowcmd_timing',lambda msg:self._string_callback('lowcmd_timing',msg),100)
        self.create_subscription(String,'/g1/policy_diagnostics',lambda msg:self._string_callback('policy_diagnostics',msg),10)
        self.create_subscription(String,'/g1/locomotion_diagnostics',lambda msg:self._string_callback('locomotion_diagnostics',msg),10)
        self.create_subscription(String,'/transport/mission_state',lambda msg:self._string_callback('transport_state',msg),10)
        self.create_subscription(String,'/exploration/search_status',lambda msg:self._string_callback('search_status',msg),10)
        self.create_subscription(TwistStamped,'/g1/policy_cmd_vel',self._policy_command_callback,10)
        if self.get_parameter('record_control_state').value:
            from unitree_hg.msg import LowCmd,LowState
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(String,'/g1/dex3_grasp_state',lambda msg:self._string_callback('dex3_grasp_state',msg),10)
            self.create_subscription(LowCmd,'/lowcmd',self._lowcmd_callback,qos_profile_sensor_data)
            self.create_subscription(LowState,'/lowstate',self._lowstate_callback,qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray,'/g1/locomotion_position_commands',
                lambda msg:self._write('actor_targets',list(msg.data)),qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray,'/g1/body_position_commands',
                lambda msg:self._write('composed_targets',list(msg.data)),qos_profile_sensor_data)
            self.create_subscription(Bool,'/g1/arm_authority',lambda msg:self._write('arm_authority',msg.data),10)

        self.get_logger().info(f"Human-aware run logger writing {self.output_path}")

    def _string_callback(self, topic: str, msg: String) -> None:
        payload: object
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = msg.data
        self._write(topic, payload)

    def _lowcmd_callback(self,msg):
        self._write('lowcmd_control',dict(role='observed_publication_not_motor_application',
            motors=[dict(q=float(m.q),dq=float(m.dq),tau=float(m.tau),kp=float(m.kp),kd=float(m.kd),mode=int(m.mode))
                    for m in msg.motor_cmd[:29]]))

    def _lowstate_callback(self,msg):
        self._write('lowstate_control',dict(device_tick_ms=int(msg.tick),
            q=[float(m.q) for m in msg.motor_state[:29]],dq=[float(m.dq) for m in msg.motor_state[:29]],
            quaternion=[float(v) for v in msg.imu_state.quaternion],gyro=[float(v) for v in msg.imu_state.gyroscope]))

    def _twist_callback(self, topic: str, msg: Twist) -> None:
        self._write(topic, self._twist_dict(msg))

    def _policy_command_callback(self,msg):
        self._write('policy_cmd',dict(self._twist_dict(msg.twist),
            source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            role='filtered_command_not_inference_ack'))

    def _odom_callback(self, msg: Odometry, topic: str = "odom") -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        self._write(
            topic,
            {
                "source_stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "frame_id": msg.header.frame_id,
                "x": round(float(pose.position.x), 4),
                "y": round(float(pose.position.y), 4),
                "z": round(float(pose.position.z), 4),
                "qx": round(float(pose.orientation.x), 6),
                "qy": round(float(pose.orientation.y), 6),
                "qz": round(float(pose.orientation.z), 6),
                "qw": round(float(pose.orientation.w), 6),
                "vx": round(float(twist.linear.x), 4),
                "vy": round(float(twist.linear.y), 4),
                "wz": round(float(twist.angular.z), 4),
            },
        )

    def _write(self, topic: str, payload: object) -> None:
        stamp = self.get_clock().now().nanoseconds * 1e-9
        self.log_file.write(json.dumps({"stamp": round(stamp, 6), "receipt_monotonic_s":time.monotonic(),
                                       "topic": topic, "data": payload}, separators=(",", ":")) + "\n")
        self.write_count += 1
        if self.write_count % self.flush_every == 0:
            self.log_file.flush()

    @staticmethod
    def _twist_dict(msg: Twist) -> dict[str, float]:
        return {
            "x": round(float(msg.linear.x), 4),
            "y": round(float(msg.linear.y), 4),
            "z": round(float(msg.angular.z), 4),
        }

    def destroy_node(self) -> bool:
        try:
            self.log_file.flush()
            self.log_file.close()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HumanAwareRunLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
