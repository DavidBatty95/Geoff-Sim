import json
import math
import threading
import time
from dataclasses import asdict,dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray,Bool,String
from unitree_hg.msg import LowState

from .g1_constants import G1_29DOF_JOINTS
from .motion_envelope import load_motion_envelope
from .policy_timing import inference_timing,result_is_stale
from .policy_safety import (
    advance_gait_phase,
    apply_command_deadband,
    axis_isolated_command,
    cleared_motion_state,
    clamp_joint_targets,
    ensure_forward_turning_arc,
    filter_command,
    pending_target,
    sample_age,
    startup_active,
    validate_metadata_joint_order,
)
from .unitree_rl_lab_policy import (
    UNITREE_RL_LAB_G1_ACTION_DIM,
    UNITREE_RL_LAB_G1_OBS_DIM,
    UnitreeRLLabG1PolicyAdapter,
)

try:
    import torch
except ImportError:  # pragma: no cover - reported at runtime on systems without torch.
    torch = None

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover - optional deployment dependency.
    ort = None


LOWER_BODY_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

LATEST_RELIABLE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

DEFAULT_JOINT_POS = {
    ".*_hip_pitch_joint": -0.312,
    ".*_knee_joint": 0.669,
    ".*_ankle_pitch_joint": -0.363,
    ".*_elbow_joint": 0.6,
    "left_shoulder_roll_joint": 0.2,
    "left_shoulder_pitch_joint": 0.2,
    "right_shoulder_roll_joint": -0.2,
    "right_shoulder_pitch_joint": 0.2,
}

ACTION_SCALE = {
    ".*_hip_pitch_joint": 0.5475464629598068,
    ".*_hip_yaw_joint": 0.5475464629598068,
    ".*_hip_roll_joint": 0.35066146635877904,
    ".*_knee_joint": 0.35066146635877904,
    ".*_ankle_pitch_joint": 0.43857731389829635,
    ".*_ankle_roll_joint": 0.43857731389829635,
    "waist_yaw_joint": 0.5475464629598068,
    "waist_roll_joint": 0.43857731389829635,
    "waist_pitch_joint": 0.43857731389829635,
    ".*_shoulder_pitch_joint": 0.43857731389829635,
    ".*_shoulder_roll_joint": 0.43857731389829635,
    ".*_shoulder_yaw_joint": 0.43857731389829635,
    ".*_elbow_joint": 0.43857731389829635,
    ".*_wrist_roll_joint": 0.43857731389829635,
    ".*_wrist_pitch_joint": 0.07450087032524835,
    ".*_wrist_yaw_joint": 0.07450087032524835,
}


def _pattern_value(name: str, patterns: dict[str, float], default: float = 0.0) -> float:
    for pattern, value in patterns.items():
        if pattern.startswith(".*") and name.endswith(pattern[2:]):
            return value
        if pattern == name:
            return value
    return default


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _parse_float_csv(value: str) -> List[float]:
    if not value:
        return []
    result = []
    for item in value.split(","):
        item = item.strip()
        if item:
            result.append(float(item))
    return result


def _parse_str_csv(value: str) -> List[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def _quat_conjugate(quat: Sequence[float]) -> tuple[float, float, float, float]:
    return quat[0], -quat[1], -quat[2], -quat[3]


def _quat_rotate(quat: Sequence[float], vec: Sequence[float]) -> List[float]:
    w, x, y, z = quat
    vx, vy, vz = vec
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    ]


@dataclass(frozen=True)
class PolicyInferenceCandidate:
    obs_dim: int
    action_dim: int
    observation: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "observation", tuple(self.observation))


@dataclass(frozen=True)
class PolicyInferenceRequest:
    generation: int
    sequence: int
    submitted_at: float
    deadline: float
    candidates: tuple[PolicyInferenceCandidate, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "candidates", tuple(self.candidates))


@dataclass(frozen=True)
class PolicyInferenceResult:
    generation: int
    sequence: int
    submitted_at: float
    completed_at: float
    obs_dim: Optional[int] = None
    action_dim: Optional[int] = None
    actions: tuple[float, ...] = ()
    error: Optional[str] = None
    deadline_missed: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "actions", tuple(self.actions))


@dataclass(frozen=True)
class PolicyInferenceDiagnostics:
    submitted: int
    started: int
    completed: int
    succeeded: int
    failed: int
    expired: int
    superseded: int
    invalidated: int
    discarded: int


class PolicyInferenceWorker:
    """Run one inference at a time without queueing stale observations."""

    def __init__(self, policy) -> None:
        self._policy = policy
        self._condition = threading.Condition()
        self._generation = 0
        self._sequence = 0
        self._pending: Optional[PolicyInferenceRequest] = None
        self._active: Optional[PolicyInferenceRequest] = None
        self._result: Optional[PolicyInferenceResult] = None
        self._stopping = False
        self._submitted = 0
        self._started = 0
        self._completed = 0
        self._succeeded = 0
        self._failed = 0
        self._expired = 0
        self._superseded = 0
        self._invalidated = 0
        self._discarded = 0
        self._thread = threading.Thread(
            target=self._run, name="g1-policy-inference", daemon=True
        )
        self._thread.start()

    @property
    def thread(self) -> threading.Thread:
        return self._thread

    def submit(
        self, candidates: Sequence[PolicyInferenceCandidate], deadline: float
    ) -> Optional[PolicyInferenceRequest]:
        with self._condition:
            if self._stopping:
                raise RuntimeError("Policy inference worker is shut down")
            if self._pending is not None or self._active is not None or self._result is not None:
                return None
            self._sequence += 1
            request = PolicyInferenceRequest(
                generation=self._generation,
                sequence=self._sequence,
                submitted_at=time.monotonic(),
                deadline=float(deadline),
                candidates=tuple(candidates),
            )
            self._pending = request
            self._submitted += 1
            self._condition.notify()
            return request

    def take_result(self) -> Optional[PolicyInferenceResult]:
        with self._condition:
            result = self._result
            self._result = None
            return result

    def has_result(self) -> bool:
        with self._condition:
            return self._result is not None

    def deadline_missed(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else now
        with self._condition:
            requests = (self._active, self._pending)
            return any(
                request is not None
                and request.generation == self._generation
                and current > request.deadline
                for request in requests
            )

    def invalidate_expired(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else now
        with self._condition:
            requests = (self._active, self._pending)
            if not any(
                request is not None
                and request.generation == self._generation
                and current > request.deadline
                for request in requests
            ):
                return False
            self._generation += 1
            self._pending = None
            self._result = None
            self._expired += 1
            self._invalidated += 1
            return True

    def invalidate(self) -> int:
        with self._condition:
            self._generation += 1
            self._pending = None
            self._result = None
            self._invalidated += 1
            return self._generation

    def diagnostics(self) -> PolicyInferenceDiagnostics:
        with self._condition:
            return PolicyInferenceDiagnostics(
                submitted=self._submitted,
                started=self._started,
                completed=self._completed,
                succeeded=self._succeeded,
                failed=self._failed,
                expired=self._expired,
                superseded=self._superseded,
                invalidated=self._invalidated,
                discarded=self._discarded,
            )

    def shutdown(self, timeout: float = 1.0) -> bool:
        with self._condition:
            if not self._stopping:
                self._stopping = True
                self._generation += 1
                self._pending = None
                self._result = None
                self._invalidated += 1
                self._condition.notify_all()
        self._thread.join(max(0.0, timeout))
        return not self._thread.is_alive()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                request = self._pending
                self._pending = None
                self._active = request
                self._started += 1

            assert request is not None
            result = self._infer(request)
            with self._condition:
                self._active = None
                self._completed += 1
                if self._stopping or request.generation != self._generation:
                    self._discarded += 1
                    continue
                if result.deadline_missed:
                    self._expired += 1
                    self._generation += 1
                    self._pending = None
                    self._invalidated += 1
                elif result.error is not None:
                    self._failed += 1
                    self._generation += 1
                    self._pending = None
                    self._invalidated += 1
                else:
                    self._succeeded += 1
                if self._result is not None:
                    self._superseded += 1
                self._result = result

    def _infer(self, request: PolicyInferenceRequest) -> PolicyInferenceResult:
        if time.monotonic() > request.deadline:
            return self._deadline_result(request)

        last_error: Optional[Exception] = None
        for candidate in request.candidates:
            if time.monotonic() > request.deadline:
                return self._deadline_result(request)
            try:
                observation = np.asarray(
                    [candidate.observation], dtype=np.float32
                )
                if observation.shape != (1, candidate.obs_dim):
                    raise ValueError(
                        f"Observation shape {observation.shape} does not match "
                        f"(1, {candidate.obs_dim})"
                    )
                actions = self._normalize_actions(
                    self._policy(observation), candidate.action_dim
                )
                completed_at = time.monotonic()
                if completed_at > request.deadline:
                    return self._deadline_result(request, completed_at)
                return PolicyInferenceResult(
                    generation=request.generation,
                    sequence=request.sequence,
                    submitted_at=request.submitted_at,
                    completed_at=completed_at,
                    obs_dim=candidate.obs_dim,
                    action_dim=candidate.action_dim,
                    actions=actions,
                )
            except Exception as exc:  # noqa: BLE001 - try the alternate shape.
                last_error = exc

        if time.monotonic() > request.deadline:
            return self._deadline_result(request)
        return PolicyInferenceResult(
            generation=request.generation,
            sequence=request.sequence,
            submitted_at=request.submitted_at,
            completed_at=time.monotonic(),
            error=str(last_error or "No policy inference candidates"),
        )

    @staticmethod
    def _deadline_result(
        request: PolicyInferenceRequest, completed_at: Optional[float] = None
    ) -> PolicyInferenceResult:
        return PolicyInferenceResult(
            generation=request.generation,
            sequence=request.sequence,
            submitted_at=request.submitted_at,
            completed_at=time.monotonic() if completed_at is None else completed_at,
            error="Policy inference deadline missed",
            deadline_missed=True,
        )

    @staticmethod
    def _normalize_actions(action, action_dim: int) -> tuple[float, ...]:
        if isinstance(action, (tuple, list)):
            action = action[0]
        if torch is not None and isinstance(action, torch.Tensor):
            values = action.detach().to("cpu").reshape(-1).tolist()
        else:
            values = np.asarray(action).reshape(-1).tolist()
        if len(values) < action_dim:
            raise ValueError(
                f"Policy returned {len(values)} actions, expected {action_dim}"
            )
        normalized = tuple(float(value) for value in values[:action_dim])
        if not all(math.isfinite(value) for value in normalized):
            raise ValueError("Policy returned non-finite actions")
        return normalized


class G1PolicyCmdVelController(Node):
    def __init__(self) -> None:
        super().__init__("g1_policy_cmd_vel_controller")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("policy_profile", "mjlab")
        self.declare_parameter("external_arm_observation_mask", False)
        self.arm_authority = False
        self.arm_authority_received = -math.inf
        self.create_subscription(Bool,'/g1/arm_authority',self._arm_authority_callback,1)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("policy_cmd_vel_topic", "/g1/policy_cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("body_command_topic", "/g1/body_position_commands")
        self.declare_parameter("cmd_vel_timeout", 0.4)
        self.declare_parameter("lowstate_timeout", 0.1)
        self.declare_parameter("odom_timeout", 0.2)
        self.declare_parameter("cmd_vel_deadband", 0.05)
        self.declare_parameter("cmd_vel_deadband_hysteresis", 0.01)
        self.declare_parameter("max_lin_vel_x", 0.35)
        self.declare_parameter("max_lin_vel_y", 0.12)
        self.declare_parameter("max_ang_vel_z", 0.35)
        self.declare_parameter("cmd_accel_x", 0.22)
        self.declare_parameter("cmd_accel_y", 0.16)
        self.declare_parameter("cmd_accel_yaw", 0.35)
        self.declare_parameter("axis_exclusive_cmd_vel", False)
        self.declare_parameter("axis_exclusive_yaw_threshold", 0.05)
        self.declare_parameter("turning_arc_min_forward_speed", 0.0)
        self.declare_parameter("turning_arc_min_yaw_speed", 0.0)
        self.declare_parameter("allow_reverse_cmd_vel", True)
        self.declare_parameter("action_clip", 1.0)
        self.declare_parameter("action_rate_limit", 0.0)
        self.declare_parameter("policy_obs_dim", 0)
        self.declare_parameter("policy_action_dim", 0)
        self.declare_parameter("gait_cycle_time", 0.8)
        self.declare_parameter("policy_inference_deadline_sec", 0.05)
        self.declare_parameter("policy_command_height", 0.84)
        self.declare_parameter("startup_blend_time", 0.0)
        self.declare_parameter("startup_hold_time", 0.0)
        self.declare_parameter("startup_current_pose_hold_time", 0.0)
        self.declare_parameter("startup_require_upright", False)
        self.declare_parameter("startup_upright_tilt_threshold", 0.35)
        self.declare_parameter("startup_max_recovery_time", 8.0)
        self.declare_parameter("policy_action_ramp_time", 0.0)
        self.declare_parameter("tilt_guard_enable", True)
        self.declare_parameter("tilt_guard_enter", 0.65)
        self.declare_parameter("tilt_guard_exit", 0.35)
        self.declare_parameter("tilt_guard_hold_time", 0.8)
        self.declare_parameter("pre_lowstate_stand_enable", False)
        self.declare_parameter("detect_robot_reset", True)
        self.declare_parameter("reset_joint_delta_threshold", 0.75)
        self.declare_parameter("reset_joint_total_delta_threshold", 4.0)
        self.declare_parameter("reset_fallen_tilt_threshold", 0.95)
        self.declare_parameter("reset_upright_tilt_threshold", 0.45)
        self.declare_parameter("reset_detection_cooldown", 1.0)
        self.declare_parameter("device", "cpu")

        self.device = str(self.get_parameter("device").value)
        self.policy_profile = str(self.get_parameter("policy_profile").value)
        if self.policy_profile not in ("mjlab", "unitree_rl_lab_g1_29dof"):
            raise ValueError(f"Unknown policy_profile: {self.policy_profile}")
        self.policy_path = self._resolve_policy_path(str(self.get_parameter("policy_path").value))
        self.policy = self._load_policy(self.policy_path)
        self.policy_obs_dim: Optional[int] = None
        self.policy_action_dim: Optional[int] = None
        policy_obs_dim = int(self.get_parameter("policy_obs_dim").value)
        policy_action_dim = int(self.get_parameter("policy_action_dim").value)
        if policy_obs_dim in (80, 82, 97, 99, 101, UNITREE_RL_LAB_G1_OBS_DIM):
            self.policy_obs_dim = policy_obs_dim
        if policy_action_dim in (12, 29):
            self.policy_action_dim = policy_action_dim
        self.policy_command_names: list[str] = []
        self.last_lowstate: Optional[LowState] = None
        self.last_lowstate_arrival: Optional[float] = None
        self.last_odom_arrival: Optional[float] = None
        self.lowstate_stale = False
        self.previous_joint_positions: Optional[List[float]] = None
        self.last_stable_positions: Optional[List[float]] = None
        self.previous_tilt: Optional[float] = None
        self.first_lowstate_time = None
        self.initial_positions: Optional[List[float]] = None
        self.upright_since = None
        self.last_reset_time = None
        self.tilt_guard_active = False
        self.tilt_guard_clear_since = None
        self.reset_count = 0
        self.last_cmd_time = self.get_clock().now()
        self.command = [0.0, 0.0, 0.0]
        self.filtered_command = [0.0, 0.0, 0.0]
        self.command_stopped = True
        self.gait_phase = 0.0
        self.last_control_time = self.get_clock().now()
        self.first_policy_time = None
        self.cmd_vel_count = 0
        self.lowstate_count = 0
        self.body_command_count = 0
        self.last_action_norm = 0.0
        self.last_active_command = [0.0, 0.0, 0.0]
        self.last_tilt = 0.0
        self.control_state = "waiting_lowstate"
        self.base_lin_vel = [0.0, 0.0, 0.0]
        self.last_safe_targets: Optional[List[float]] = None
        self.body_joint_names = list(G1_29DOF_JOINTS)
        self.unitree_rl_lab_adapter = (
            UnitreeRLLabG1PolicyAdapter()
            if self.policy_profile == "unitree_rl_lab_g1_29dof"
            else None
        )
        self.lower_joint_indexes = [
            self.body_joint_names.index(j) for j in LOWER_BODY_JOINTS
        ]
        self.default_positions = [
            _pattern_value(name, DEFAULT_JOINT_POS) for name in self.body_joint_names
        ]
        self.scales = [
            _pattern_value(name, ACTION_SCALE, 0.25) for name in self.body_joint_names
        ]
        if self.unitree_rl_lab_adapter is not None:
            self.policy_obs_dim = UNITREE_RL_LAB_G1_OBS_DIM
            self.policy_action_dim = UNITREE_RL_LAB_G1_ACTION_DIM
            self.default_positions = self.unitree_rl_lab_adapter.sdk_default_positions
            self.scales = [0.25] * len(self.body_joint_names)
        self.last_actions = {
            12: [0.0] * len(self.lower_joint_indexes),
            29: [0.0] * len(self.body_joint_names),
        }
        self._apply_policy_metadata()
        self._log_policy_configuration()
        self.policy_worker = PolicyInferenceWorker(self.policy)
        self.policy_inference_deadline = max(
            0.001,
            float(self.get_parameter("policy_inference_deadline_sec").value),
        )

        self.body_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("body_command_topic").value),
            LATEST_RELIABLE_QOS,
        )
        self.policy_cmd_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("policy_cmd_vel_topic").value),
            LATEST_RELIABLE_QOS,
        )
        self.policy_timing_pub = self.create_publisher(String,'/g1/policy_timing',10)
        self.policy_diagnostics_pub = self.create_publisher(String,'/g1/policy_diagnostics',10)
        self.inference_context = None
        self.stop_request_invalidations = 0
        self.stale_result_rejections = 0
        self.create_subscription(
            LowState, "/lowstate", self._lowstate_callback, LATEST_RELIABLE_QOS
        )
        self.declare_parameter('legacy_rt_aliases',False)
        if self.get_parameter('legacy_rt_aliases').value:
            self.create_subscription(LowState,"rt/lowstate",self._lowstate_callback,LATEST_RELIABLE_QOS)
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._cmd_vel_callback,
            LATEST_RELIABLE_QOS,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            LATEST_RELIABLE_QOS,
        )

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.control_period = period
        self.create_timer(period, self._control_tick)
        # Inference cadence stays at the policy rate; collect finished work
        # promptly instead of inserting a whole extra policy-period delay.
        self.create_timer(min(.002,period), self._result_tick)
        self.create_timer(2.0, self._diagnostics)
        self.get_logger().info("G1 policy /cmd_vel controller ready")

    def _log_policy_configuration(self) -> None:
        command_mode = "height" if self._uses_height_command() else "velocity"
        action_scope = "lower-body" if self.policy_action_dim == 12 else "whole-body" if self.policy_action_dim == 29 else "unknown"
        self.get_logger().info(
            "Loaded G1 policy: "
            f"path={self.policy_path} profile={self.policy_profile} obs_dim={self.policy_obs_dim} "
            f"action_dim={self.policy_action_dim} action_scope={action_scope} command_mode={command_mode} "
            f"command_names={self.policy_command_names or 'unknown'} "
            f"target_height={float(self.get_parameter('policy_command_height').value):.3f}"
        )

    def _load_policy(self, policy_path: str):
        if not policy_path:
            raise ValueError(
                "policy_path must point to a TorchScript, ONNX, or callable PyTorch policy"
            )

        path = Path(policy_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Policy file does not exist: {path}")

        if path.suffix == ".onnx":
            if ort is None:
                raise RuntimeError("onnxruntime is required to run ONNX policies")
            return OnnxPolicy(str(path))

        if torch is None:
            raise RuntimeError("PyTorch is required to run TorchScript/PyTorch policies")

        try:
            policy = torch.jit.load(str(path), map_location=self.device)
        except RuntimeError:
            policy = torch.load(str(path), map_location=self.device)
            if isinstance(policy, dict):
                for key in ("policy", "model", "actor"):
                    if key in policy and callable(policy[key]):
                        policy = policy[key]
                        break
        if not callable(policy):
            raise TypeError(
                "Policy file must be TorchScript or contain a callable policy/model/actor"
            )
        return TorchPolicy(policy, self.device)

    def _resolve_policy_path(self, policy_path: str) -> str:
        path = policy_path.strip()
        if path and path.lower() not in ("latest", "auto"):
            return path

        roots = [
            Path("/workspace/vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop"),
            Path.cwd() / "vendor/mjlab/logs/rsl_rl/g1_stand_creep_stop",
            Path("/workspace/vendor/mjlab/logs/rsl_rl/g1_velocity"),
            Path.cwd() / "vendor/mjlab/logs/rsl_rl/g1_velocity",
        ]
        preferred: list[Path] = []
        fallback: list[Path] = []
        for root in roots:
            if not root.exists():
                continue
            if root.name == "g1_stand_creep_stop":
                preferred.extend(root.glob("*/*.onnx"))
                continue
            preferred.extend(root.glob("*g1_flat_locomotion/*.onnx"))
            preferred.extend(root.glob("*flat_locomotion/*.onnx"))
            fallback.extend(root.glob("*g1_flat_lower_body_locomotion/*.onnx"))
            fallback.extend(root.glob("*lower_body_locomotion/*.onnx"))

        candidates = [candidate for candidate in preferred if candidate.is_file()]
        if not candidates:
            candidates = [candidate for candidate in fallback if candidate.is_file()]
        if candidates:
            latest = max(candidates, key=lambda candidate: candidate.stat().st_mtime)
            self.get_logger().info(f"Resolved latest G1 walking policy: {latest}")
            return str(latest)

        raise FileNotFoundError(
            "No G1 walking policy ONNX was found. Train/export a stand/creep/stop or "
            "flat locomotion policy into vendor/mjlab/logs/rsl_rl, or launch with "
            "policy_path:=/workspace/path/to/policy.onnx."
        )

    def _apply_policy_metadata(self) -> None:
        metadata = getattr(self.policy, "metadata", {})
        if not metadata:
            return

        default_joint_pos = _parse_float_csv(metadata.get("default_joint_pos", ""))
        action_scale = _parse_float_csv(metadata.get("action_scale", ""))
        joint_names = _parse_str_csv(metadata.get("joint_names", ""))
        self.policy_command_names = _parse_str_csv(metadata.get("command_names", ""))
        metadata_action_dim = self.policy_action_dim or getattr(
            self.policy, "action_dim", None
        )
        validate_metadata_joint_order(
            joint_names,
            self.body_joint_names,
            LOWER_BODY_JOINTS,
            metadata_action_dim,
        )
        if len(default_joint_pos) == len(self.body_joint_names):
            self.default_positions = default_joint_pos
        elif len(default_joint_pos) == len(self.lower_joint_indexes):
            if joint_names and len(joint_names) == len(default_joint_pos):
                for name, value in zip(joint_names, default_joint_pos):
                    if name in self.body_joint_names:
                        self.default_positions[self.body_joint_names.index(name)] = value
            else:
                for value, joint_index in zip(default_joint_pos, self.lower_joint_indexes):
                    self.default_positions[joint_index] = value
        if len(action_scale) == len(self.lower_joint_indexes):
            if joint_names and len(joint_names) == len(action_scale):
                for name, value in zip(joint_names, action_scale):
                    if name in self.body_joint_names:
                        self.scales[self.body_joint_names.index(name)] = value
            else:
                for action_index, joint_index in enumerate(self.lower_joint_indexes):
                    self.scales[joint_index] = action_scale[action_index]
        elif len(action_scale) == len(self.body_joint_names):
            self.scales = action_scale

        obs_dim = getattr(self.policy, "obs_dim", None)
        action_dim = getattr(self.policy, "action_dim", None)
        if self.policy_obs_dim is None and obs_dim in (80, 82, 97, 99, 101, UNITREE_RL_LAB_G1_OBS_DIM):
            self.policy_obs_dim = int(obs_dim)
        if self.policy_action_dim is None and action_dim in (12, 29):
            self.policy_action_dim = int(action_dim)

    def _lowstate_callback(self, msg: LowState) -> None:
        if len(msg.motor_state) < len(self.body_joint_names):
            self.get_logger().warn(
                f"Ignoring short lowstate with {len(msg.motor_state)} motor states",
                throttle_duration_sec=2.0,
            )
            return
        if (
            len(msg.imu_state.rpy) < 2
            or len(msg.imu_state.quaternion) < 4
            or len(msg.imu_state.gyroscope) < 3
        ):
            self.get_logger().warn(
                "Ignoring lowstate with incomplete IMU data", throttle_duration_sec=2.0
            )
            return
        joint_positions = [float(state.q) for state in msg.motor_state[:29]]
        joint_velocities = [float(state.dq) for state in msg.motor_state[:29]]
        imu_values = [float(value) for value in msg.imu_state.rpy[:2]]
        imu_values.extend(float(value) for value in msg.imu_state.quaternion[:4])
        imu_values.extend(float(value) for value in msg.imu_state.gyroscope[:3])
        if not all(math.isfinite(value) for value in joint_positions + joint_velocities + imu_values):
            self.get_logger().warn(
                "Ignoring lowstate containing non-finite values", throttle_duration_sec=2.0
            )
            return
        self.lowstate_count += 1
        tilt = self._lowstate_tilt(msg)
        self.last_tilt = tilt
        if self._detect_robot_reset(joint_positions, tilt):
            self._reset_controller_state(joint_positions, "reset/upright transition")
        if self.first_lowstate_time is None:
            self.first_lowstate_time = self.get_clock().now()
            self.initial_positions = list(joint_positions)
        if tilt < float(self.get_parameter("startup_upright_tilt_threshold").value):
            self.last_stable_positions = list(joint_positions)
        self._update_upright_state(tilt)
        self.previous_joint_positions = list(joint_positions)
        self.previous_tilt = tilt
        self.last_lowstate = msg
        self.last_lowstate_arrival = time.monotonic()
        self.lowstate_stale = False

    def _tilt_guard_holding(self) -> bool:
        if not bool(self.get_parameter("tilt_guard_enable").value):
            return False
        now = self.get_clock().now()
        enter = float(self.get_parameter("tilt_guard_enter").value)
        exit_threshold = float(self.get_parameter("tilt_guard_exit").value)
        hold_time = float(self.get_parameter("tilt_guard_hold_time").value)
        if self.last_tilt >= enter:
            if not self.tilt_guard_active:
                self.get_logger().warn(
                    f"Tilt guard holding stable joint pose (tilt={self.last_tilt:.2f} rad)",
                    throttle_duration_sec=1.0,
                )
                self.policy_worker.invalidate()
                if self.unitree_rl_lab_adapter is not None:
                    self.unitree_rl_lab_adapter.reset()
            self.tilt_guard_active = True
            self.tilt_guard_clear_since = None
            self.filtered_command = [0.0, 0.0, 0.0]
            self.first_policy_time = None
            self.last_actions = {
                12: [0.0] * len(self.lower_joint_indexes),
                29: [0.0] * len(self.body_joint_names),
            }
            self.last_action_norm = 0.0
            return True
        if not self.tilt_guard_active:
            return False
        if self.last_tilt <= exit_threshold:
            if self.tilt_guard_clear_since is None:
                self.tilt_guard_clear_since = now
            stable_time = (now - self.tilt_guard_clear_since).nanoseconds * 1e-9
            if stable_time >= hold_time:
                self.tilt_guard_active = False
                self.tilt_guard_clear_since = None
                self.first_policy_time = None
                return False
        else:
            self.tilt_guard_clear_since = None
        return True

    def _update_upright_state(self, tilt: float) -> None:
        if tilt < float(self.get_parameter("startup_upright_tilt_threshold").value):
            if self.upright_since is None:
                self.upright_since = self.get_clock().now()
        else:
            self.upright_since = None

    def _detect_robot_reset(self, joint_positions: Sequence[float], tilt: float) -> bool:
        if not bool(self.get_parameter("detect_robot_reset").value):
            return False
        now = self.get_clock().now()
        if self.last_reset_time is not None:
            cooldown = float(self.get_parameter("reset_detection_cooldown").value)
            if (now - self.last_reset_time).nanoseconds * 1e-9 < cooldown:
                return False

        if self.previous_tilt is not None:
            if (
                self.previous_tilt > float(self.get_parameter("reset_fallen_tilt_threshold").value)
                and tilt < float(self.get_parameter("reset_upright_tilt_threshold").value)
            ):
                return True

        if self.previous_joint_positions is None:
            return False

        deltas = [abs(current - previous) for current, previous in zip(joint_positions, self.previous_joint_positions)]
        if not deltas:
            return False
        max_delta = max(deltas)
        total_delta = sum(deltas)
        return (
            max_delta > float(self.get_parameter("reset_joint_delta_threshold").value)
            and total_delta > float(self.get_parameter("reset_joint_total_delta_threshold").value)
        )

    @staticmethod
    def _lowstate_tilt(msg: LowState) -> float:
        rpy = msg.imu_state.rpy
        roll = math.atan2(math.sin(float(rpy[0])), math.cos(float(rpy[0])))
        pitch = math.atan2(math.sin(float(rpy[1])), math.cos(float(rpy[1])))
        return max(abs(roll), abs(pitch))

    def _reset_controller_state(self, joint_positions: Sequence[float], reason: str) -> None:
        now = self.get_clock().now()
        self.policy_worker.invalidate()
        self.first_lowstate_time = now
        self.initial_positions = list(joint_positions)
        self.last_stable_positions = list(joint_positions)
        self.upright_since = None
        self.filtered_command = [0.0, 0.0, 0.0]
        self.command = [0.0, 0.0, 0.0]
        self.command_stopped = True
        self.gait_phase = 0.0
        self.last_active_command = [0.0, 0.0, 0.0]
        (
            self.base_lin_vel,
            self.last_lowstate_arrival,
            self.last_odom_arrival,
        ) = cleared_motion_state()
        self.last_cmd_time = now
        self.last_control_time = now
        self.first_policy_time = None
        self.last_actions = {
            12: [0.0] * len(self.lower_joint_indexes),
            29: [0.0] * len(self.body_joint_names),
        }
        if self.unitree_rl_lab_adapter is not None:
            self.unitree_rl_lab_adapter.reset()
        self.last_action_norm = 0.0
        self.last_reset_time = now
        self.reset_count += 1
        self.get_logger().warn(
            f"Detected robot reset from {reason}; restarting policy startup blend",
            throttle_duration_sec=1.0,
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        values = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn(
                "Ignoring cmd_vel containing non-finite values", throttle_duration_sec=2.0
            )
            return
        self.cmd_vel_count += 1
        self.last_cmd_time = self.get_clock().now()
        self.command = [
            _clamp(values[0], float(self.get_parameter("max_lin_vel_x").value)),
            _clamp(values[1], float(self.get_parameter("max_lin_vel_y").value)),
            _clamp(values[2], float(self.get_parameter("max_ang_vel_z").value)),
        ]
        if not bool(self.get_parameter("allow_reverse_cmd_vel").value):
            self.command[0] = max(0.0, self.command[0])

    def _odom_callback(self, msg: Odometry) -> None:
        values = [
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        ]
        if all(math.isfinite(value) for value in values):
            self.base_lin_vel = values
            self.last_odom_arrival = time.monotonic()

    def _active_command(self) -> tuple[List[float], bool]:
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > float(self.get_parameter("cmd_vel_timeout").value):
            self.command_stopped = True
            return [0.0, 0.0, 0.0], True
        command, self.command_stopped = apply_command_deadband(
            self.command,
            self.command_stopped,
            float(self.get_parameter("cmd_vel_deadband").value),
            float(self.get_parameter("cmd_vel_deadband_hysteresis").value),
        )
        return command, self.command_stopped

    def _filtered_active_command(
        self, target: Sequence[float], dt: float, immediate_stop: bool = False
    ) -> List[float]:
        envelope = load_motion_envelope(self)
        target = envelope.normalize(target, stopped=self.command_stopped)
        turning = False
        if bool(self.get_parameter("axis_exclusive_cmd_vel").value):
            target, turning = axis_isolated_command(
                target,
                float(self.get_parameter("axis_exclusive_yaw_threshold").value),
            )
        limits = [
            float(self.get_parameter("cmd_accel_x").value),
            float(self.get_parameter("cmd_accel_y").value),
            float(self.get_parameter("cmd_accel_yaw").value),
        ]
        previous = self.filtered_command
        self.filtered_command = filter_command(
            self.filtered_command, target, limits, dt, immediate_stop
        )
        self.filtered_command = list(envelope.limit_planar(self.filtered_command,previous))
        if bool(self.get_parameter("axis_exclusive_cmd_vel").value):
            if turning:
                self.filtered_command[0] = 0.0
                self.filtered_command[1] = 0.0
            else:
                self.filtered_command[2] = 0.0
        return list(self.filtered_command)

    def _publish_policy_command(self, command: Sequence[float]) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = float(command[0])
        message.twist.linear.y = float(command[1])
        message.twist.angular.z = float(command[2])
        self.policy_cmd_pub.publish(message)

    def _result_tick(self) -> None:
        if self.policy_worker.has_result():
            self._control_tick(consume_only=True)

    def _control_tick(self, consume_only: bool = False) -> None:
        if self.last_lowstate is None:
            if bool(self.get_parameter("pre_lowstate_stand_enable").value):
                self._publish_targets(self.default_positions)
            self.get_logger().warn(
                "Waiting for /lowstate before releasing MuJoCo startup hold",
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()
        monotonic_now = time.monotonic()
        lowstate_age = sample_age(monotonic_now, self.last_lowstate_arrival)
        if lowstate_age > float(self.get_parameter("lowstate_timeout").value):
            if not self.lowstate_stale:
                self.policy_worker.invalidate()
                self.lowstate_stale = True
            self.get_logger().error(
                f"Lowstate stale ({lowstate_age:.3f}s); holding last safe target",
                throttle_duration_sec=1.0,
            )
            self._policy_safe_hold("stale_lowstate_hold")
            return
        odom_age = sample_age(monotonic_now, self.last_odom_arrival)
        if odom_age > float(self.get_parameter("odom_timeout").value):
            self.base_lin_vel = [0.0, 0.0, 0.0]
        dt = max(1e-3, min(0.1, (now - self.last_control_time).nanoseconds * 1e-9))
        if consume_only:
            dt = self.control_period
        if not consume_only:
            self.last_control_time = now
        active_command, immediate_stop = self._active_command()
        if consume_only:
            # Do not advance filters/history or submit a new observation on a
            # result wakeup. A newly received stop must still revoke old work.
            command = [0.,0.,0.] if immediate_stop else list(self.last_active_command)
        else:
            command = self._filtered_active_command(active_command, dt, immediate_stop)
        self._invalidate_stopped_inference(self.last_active_command,command,immediate_stop)
        self.last_active_command = command
        if self._startup_active():
            self._publish_policy_command((0.0, 0.0, 0.0))
            self.gait_phase = 0.0
            self.control_state = "startup_hold"
            self._publish_startup_pose()
            return
        if self._tilt_guard_holding():
            self._publish_policy_command((0.0, 0.0, 0.0))
            self.gait_phase = 0.0
            self.control_state = "tilt_guard_hold"
            self._publish_targets(self._safe_hold_positions())
            return

        result = self.policy_worker.take_result()
        if result is not None and result_is_stale(result,time.monotonic(),self.policy_inference_deadline):
            self.policy_worker.invalidate()
            self.inference_context = None
            self.stale_result_rejections += 1
            self._publish_policy_command((0.0,0.0,0.0))
            self._policy_safe_hold('policy_result_stale_hold')
            return
        if result is not None and (result.error is not None or result.deadline_missed):
            self.get_logger().error(
                result.error or "Policy inference deadline missed",
                throttle_duration_sec=2.0,
            )
            state = "policy_deadline_hold" if result.deadline_missed else "policy_error_hold"
            self._publish_policy_command((0.0, 0.0, 0.0))
            self._policy_safe_hold(state)
            return
        if self.policy_worker.invalidate_expired():
            self.get_logger().error(
                "Policy inference deadline missed", throttle_duration_sec=2.0
            )
            self._publish_policy_command((0.0, 0.0, 0.0))
            self._policy_safe_hold("policy_deadline_hold")
            return

        if result is not None:
            assert result.obs_dim is not None and result.action_dim is not None
            self.policy_obs_dim = result.obs_dim
            self.policy_action_dim = result.action_dim
            if self.first_policy_time is None:
                self.first_policy_time = now
            self.control_state = "policy"
            ramp_time = max(
                0.0, float(self.get_parameter("policy_action_ramp_time").value)
            )
            if ramp_time > 0.0:
                ramp = min(
                    1.0,
                    (now - self.first_policy_time).nanoseconds * 1e-9 / ramp_time,
                )
            else:
                ramp = 1.0
            try:
                self._publish_action(
                    np.asarray(result.actions, dtype=np.float32),
                    result.action_dim,
                    dt,
                    ramp,
                )
            except Exception as exc:  # noqa: BLE001 - hold safely on output handling errors.
                self.policy_worker.invalidate()
                self.get_logger().error(
                    f"Policy action handling failed: {exc}",
                    throttle_duration_sec=2.0,
                )
                self._publish_policy_command((0.0, 0.0, 0.0))
                self._policy_safe_hold("policy_error_hold")
                return
            if self.policy_timing_pub.get_subscription_count():
                timing = inference_timing(result,self.inference_context,time.monotonic(),
                                          self.get_clock().now().nanoseconds*1e-9)
                self.policy_timing_pub.publish(String(data=json.dumps(timing,separators=(',',':'))))

        if consume_only:
            return

        self.gait_phase = advance_gait_phase(
            self.gait_phase,
            command,
            dt,
            float(self.get_parameter("gait_cycle_time").value),
        )
        self._publish_policy_command(command)
        candidates = self._build_inference_candidates(command)
        request = self.policy_worker.submit(
            candidates, deadline=time.monotonic() + self.policy_inference_deadline
        )
        if request is not None:
            self.inference_context = dict(key=(request.generation,request.sequence),command=list(command),
                command_received_ros_s=self.last_cmd_time.nanoseconds*1e-9,
                lowstate_receipt_age_at_submit_s=sample_age(request.submitted_at,self.last_lowstate_arrival))
        if result is None:
            self.control_state = "policy_wait"
            self._publish_targets(
                pending_target(self.last_safe_targets, self._safe_hold_positions())
            )

    def _invalidate_stopped_inference(self,previous,command,immediate_stop):
        if immediate_stop and any(previous) and not any(command):
            # Both completed-but-unconsumed and running pre-stop work belong to
            # the old generation. Keep the last validated targets while a fresh
            # zero-command inference is prepared; never drop motor gains here.
            self.policy_worker.invalidate()
            self.inference_context = None
            self.stop_request_invalidations += 1

    def _policy_safe_hold(self, state: str) -> None:
        self.control_state = state
        self.filtered_command = [0.0, 0.0, 0.0]
        self.gait_phase = 0.0
        self.last_actions = {
            12: [0.0] * len(self.lower_joint_indexes),
            29: [0.0] * len(self.body_joint_names),
        }
        if self.unitree_rl_lab_adapter is not None:
            self.unitree_rl_lab_adapter.reset()
        self.last_action_norm = 0.0
        self.first_policy_time = None
        self._publish_targets(self._safe_hold_positions())

    def _build_inference_candidates(
        self, command: Sequence[float]
    ) -> tuple[PolicyInferenceCandidate, ...]:
        obs_dims = (
            (self.policy_obs_dim,)
            if self.policy_obs_dim is not None
            else (80, 82, 97, 99, 101)
        )
        candidates = []
        for obs_dim in obs_dims:
            action_dim = self.policy_action_dim or (
                12 if obs_dim in (80, 82) else 29
            )
            observation = self._build_observation(command, action_dim, obs_dim)
            candidates.append(
                PolicyInferenceCandidate(
                    obs_dim=obs_dim,
                    action_dim=action_dim,
                    observation=tuple(float(value) for value in observation.reshape(-1)),
                )
            )
        return tuple(candidates)

    def _startup_active(self) -> bool:
        if self.first_lowstate_time is None:
            return True
        # Once policy control is established, transient walking/avoidance tilt
        # belongs to the tilt guard. Re-entering startup at its lower upright
        # threshold abruptly replaced the balance policy with a neutral pose.
        # Reset/error/tilt-guard paths explicitly clear first_policy_time.
        if self.first_policy_time is not None:
            return False
        elapsed = (self.get_clock().now() - self.first_lowstate_time).nanoseconds * 1e-9
        upright_duration = None
        if self.upright_since is not None:
            upright_duration = (
                self.get_clock().now() - self.upright_since
            ).nanoseconds * 1e-9
        active = startup_active(
            elapsed,
            bool(self.get_parameter("startup_require_upright").value),
            upright_duration,
            float(self.get_parameter("startup_current_pose_hold_time").value),
            float(self.get_parameter("startup_blend_time").value),
            float(self.get_parameter("startup_hold_time").value),
        )
        if active and upright_duration is None:
            max_recovery = float(
                self.get_parameter("startup_max_recovery_time").value
            )
            timeout_text = (
                " (recovery timeout exceeded; policy remains inhibited)"
                if max_recovery > 0.0 and elapsed > max_recovery
                else ""
            )
            self.get_logger().warn(
                "Holding stable joint pose until robot is upright before enabling policy inference"
                + timeout_text,
                throttle_duration_sec=1.0,
            )
        return active

    def _publish_startup_pose(self) -> None:
        if self.first_lowstate_time is None or self.initial_positions is None:
            return
        elapsed = (self.get_clock().now() - self.first_lowstate_time).nanoseconds * 1e-9
        current_hold = max(0.0, float(self.get_parameter("startup_current_pose_hold_time").value))
        if elapsed < current_hold:
            self._publish_targets(self._safe_hold_positions())
            return
        elapsed -= current_hold
        blend_time = max(float(self.get_parameter("startup_blend_time").value), 1e-6)
        alpha = min(1.0, max(0.0, elapsed / blend_time))
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        targets = [
            (1.0 - alpha) * start + alpha * target
            for start, target in zip(self.initial_positions, self.default_positions)
        ]
        self._publish_targets(targets)

    def _safe_hold_positions(self) -> List[float]:
        if self.last_stable_positions is not None:
            return list(self.last_stable_positions)
        if self.initial_positions is not None:
            return list(self.initial_positions)
        return list(self.default_positions)

    def _diagnostics(self) -> None:
        cmd_age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        now = time.monotonic()
        lowstate_age = sample_age(now, self.last_lowstate_arrival)
        odom_age = sample_age(now, self.last_odom_arrival)
        inference = self.policy_worker.diagnostics()
        if self.policy_diagnostics_pub.get_subscription_count():
            self.policy_diagnostics_pub.publish(String(data=json.dumps(dict(
                stamp=self.get_clock().now().nanoseconds*1e-9,state=self.control_state,
                counters=asdict(inference),body_command_count=self.body_command_count,
                stop_request_invalidations=self.stop_request_invalidations,
                stale_result_rejections=self.stale_result_rejections,
                inference_deadline_s=self.policy_inference_deadline,
                control_period_s=self.control_period),separators=(',',':'))))
        self.get_logger().info(
            "policy status: "
            f"state={self.control_state} lowstate={self.lowstate_count} cmd_vel={self.cmd_vel_count} "
            f"body_cmd={self.body_command_count} resets={self.reset_count} cmd_age={cmd_age:.2f}s "
            f"lowstate_age={lowstate_age:.3f}s odom_age={odom_age:.3f}s "
            f"tilt={self.last_tilt:.2f} "
            f"cmd=({self.last_active_command[0]:+.2f}, "
            f"{self.last_active_command[1]:+.2f}, {self.last_active_command[2]:+.2f}) "
            f"action_norm={self.last_action_norm:.3f} "
            f"inference=(submitted={inference.submitted} started={inference.started} "
            f"completed={inference.completed} "
            f"ok={inference.succeeded} failed={inference.failed} expired={inference.expired} "
            f"superseded={inference.superseded} invalidated={inference.invalidated} "
            f"discarded={inference.discarded})",
            throttle_duration_sec=2.0,
        )

    def _arm_authority_callback(self,msg):
        self.arm_authority = msg.data
        self.arm_authority_received = time.monotonic()

    def _build_observation(
        self, command: Sequence[float], action_dim: int, obs_dim: Optional[int] = None
    ):
        assert self.last_lowstate is not None
        joint_pos = [float(state.q) for state in self.last_lowstate.motor_state[:29]]
        joint_vel = [float(state.dq) for state in self.last_lowstate.motor_state[:29]]
        imu = self.last_lowstate.imu_state
        if self.unitree_rl_lab_adapter is not None:
            if action_dim != UNITREE_RL_LAB_G1_ACTION_DIM or obs_dim != UNITREE_RL_LAB_G1_OBS_DIM:
                raise ValueError("Unitree RL Lab G1 profile requires a 480/29 policy")
            return self.unitree_rl_lab_adapter.build_observation(
                command,
                joint_pos,
                joint_vel,
                imu.gyroscope,
                imu.quaternion,
                self.last_actions[UNITREE_RL_LAB_G1_ACTION_DIM],
                external_arms=(self.get_parameter('external_arm_observation_mask').value and
                               self.arm_authority and time.monotonic()-self.arm_authority_received < .5),
            )
        quat = list(imu.quaternion)
        norm = math.sqrt(sum(q * q for q in quat))
        if norm > 1e-6:
            quat = [q / norm for q in quat]
        projected_gravity = _quat_rotate(_quat_conjugate(quat), (0.0, 0.0, -1.0))
        joint_pos_rel = [q - d for q, d in zip(joint_pos, self.default_positions)]

        values: list[float] = []
        values.extend(self.base_lin_vel)
        values.extend(float(v) for v in imu.gyroscope)
        values.extend(projected_gravity)
        values.extend(joint_pos_rel)
        values.extend(joint_vel)
        values.extend(self.last_actions[action_dim])
        if self._uses_height_command(obs_dim):
            values.append(float(self.get_parameter("policy_command_height").value))
        else:
            values.extend(command)
        if obs_dim == 101:
            angle = 2.0 * math.pi * self.gait_phase
            values.extend((math.sin(angle), math.cos(angle)))
        if obs_dim is not None and len(values) != obs_dim:
            raise ValueError(
                f"Built {len(values)} observations for a policy expecting {obs_dim}"
            )
        return np.asarray([values], dtype=np.float32)

    def _uses_height_command(self, obs_dim: Optional[int] = None) -> bool:
        if self.policy_command_names:
            return self.policy_command_names == ["height"]
        dimension = self.policy_obs_dim if obs_dim is None else obs_dim
        return dimension in (80, 97)

    def _publish_action(self, action, action_dim: int, dt: float, ramp: float = 1.0) -> None:
        if isinstance(action, (tuple, list)):
            action = action[0]
        if torch is not None and isinstance(action, torch.Tensor):
            action_values = action.detach().to("cpu").reshape(-1).tolist()
        else:
            action_values = np.asarray(action).reshape(-1).tolist()
        if len(action_values) < action_dim:
            raise ValueError(
                f"Policy returned {len(action_values)} actions, expected {action_dim}"
            )
        action_values = action_values[:action_dim]
        if not all(math.isfinite(float(value)) for value in action_values):
            raise ValueError("Policy returned non-finite actions")
        clip = float(self.get_parameter("action_clip").value)
        if self.unitree_rl_lab_adapter is None and clip > 0.0:
            action_values = [_clamp(float(value), clip) for value in action_values]
        else:
            action_values = [float(value) for value in action_values]
        ramp = max(0.0, min(1.0, ramp))
        if ramp < 1.0:
            action_values = [value * ramp for value in action_values]
        rate_limit = max(0.0, float(self.get_parameter("action_rate_limit").value))
        if rate_limit > 0.0:
            max_delta = rate_limit * dt
            previous = self.last_actions[action_dim]
            action_values = [
                previous[index] + _clamp(value - previous[index], max_delta)
                for index, value in enumerate(action_values)
            ]
        self.last_actions[action_dim] = action_values
        self.last_action_norm = math.sqrt(sum(value * value for value in action_values))

        targets = list(self.default_positions)
        if self.unitree_rl_lab_adapter is not None:
            if action_dim != UNITREE_RL_LAB_G1_ACTION_DIM:
                raise ValueError("Unitree RL Lab G1 profile requires 29 actions")
            targets = self.unitree_rl_lab_adapter.action_targets(action_values)
        elif action_dim == 12:
            for action_index, joint_index in enumerate(self.lower_joint_indexes):
                targets[joint_index] += (
                    action_values[action_index] * self.scales[joint_index]
                )
        else:
            for joint_index, value in enumerate(action_values):
                targets[joint_index] += value * self.scales[joint_index]

        self._publish_targets(targets)

    def _publish_targets(self, targets: Sequence[float]) -> None:
        safe_targets = clamp_joint_targets(targets)
        msg = Float64MultiArray()
        msg.data = safe_targets
        self.body_pub.publish(msg)
        self.last_safe_targets = safe_targets
        self.body_command_count += 1

    def destroy_node(self):
        if hasattr(self, "policy_worker"):
            self.policy_worker.shutdown(timeout=1.0)
        return super().destroy_node()


class OnnxPolicy:
    def __init__(self, path: str) -> None:
        assert ort is not None
        # Bound deployment CPU usage for this small control MLP. An unrestricted
        # per-core spinning pool can compete with ROS/TF between control ticks.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        options.add_session_config_entry("session.inter_op.allow_spinning", "0")
        self.session = ort.InferenceSession(path, sess_options=options, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.metadata = self.session.get_modelmeta().custom_metadata_map
        self.obs_dim = self._shape_dim(self.session.get_inputs()[0].shape, 1)
        self.action_dim = self._shape_dim(self.session.get_outputs()[0].shape, 1)

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.session.run([self.output_name], {self.input_name: obs})[0]

    @staticmethod
    def _shape_dim(shape: Sequence[object], index: int) -> Optional[int]:
        if len(shape) <= index:
            return None
        value = shape[index]
        return int(value) if isinstance(value, int) else None


class TorchPolicy:
    def __init__(self, policy, device: str) -> None:
        self.policy = policy
        self.device = device
        self.metadata = {}
        if hasattr(self.policy, "eval"):
            self.policy.eval()

    def __call__(self, obs: np.ndarray):
        assert torch is not None
        obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        with torch.no_grad():
            return self.policy(obs_tensor)


def main() -> None:
    rclpy.init()
    node = G1PolicyCmdVelController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
