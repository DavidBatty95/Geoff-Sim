"""Shared executable command bounds and response-aware planar rollouts.

Response defaults are conservative engineering assumptions, not a fitted G1
response certificate. Keep them explicit so calibration can replace them.
"""

import math
from dataclasses import dataclass

from .policy_safety import apply_command_deadband, axis_isolated_command, ensure_forward_turning_arc
from .policy_response import NATIVE_CONTACT_PROFILE,pivot_yaw_rate


def advance_planar_pose(pose, velocity, duration):
    """Advance a stamped planar pose using a constant body-frame twist."""
    x, y, yaw = pose
    vx, vy, wz = velocity
    angle = wz*duration
    if abs(wz) < 1e-6:
        dx, dy = vx*duration, vy*duration
    else:
        sine, cosine = math.sin(angle), math.cos(angle)
        dx = (vx*sine + vy*(cosine-1.))/wz
        dy = (vx*(1.-cosine) + vy*sine)/wz
    c, s = math.cos(yaw), math.sin(yaw)
    return x+c*dx-s*dy, y+s*dx+c*dy, yaw+angle


@dataclass(frozen=True)
class MotionEnvelope:
    max_x: float = 0.35
    max_y: float = 0.12
    max_yaw: float = 0.35
    accel_x: float = 0.22
    accel_y: float = 0.16
    accel_yaw: float = 0.35
    deadband: float = 0.05
    hysteresis: float = 0.01
    arc_forward: float = 0.0
    arc_yaw: float = 0.0
    yaw_threshold: float = 0.05
    axis_exclusive: bool = False
    allow_reverse: bool = True
    forward_gain: float = 1.0
    lateral_gain: float = 1.0
    yaw_gain: float = 1.0
    linear_time_constant: float = 0.25
    yaw_time_constant: float = 0.40
    max_planar_speed: float = math.inf
    lateral_yaw_gain: float = 0.0
    walking_yaw_gain: float = 0.0
    negative_lateral_gain: float = 0.0
    positive_lateral_time_constant: float = 0.0
    response_profile: str = 'legacy'
    payload_active: bool = False

    def lateral_response_gain(self, command):
        return self.negative_lateral_gain if command < 0. and self.negative_lateral_gain > 0. else self.lateral_gain

    def limit_planar(self,command,previous=None):
        values = list(command)
        limit = max(0.,self.max_planar_speed)
        speed = math.hypot(values[0],values[1])
        if speed > limit:
            if previous is not None and math.hypot(previous[0],previous[1]) <= limit+1e-9:
                dx,dy = values[0]-previous[0],values[1]-previous[1]
                length = dx*dx+dy*dy
                dot = previous[0]*dx+previous[1]*dy
                discriminant = dot*dot+length*(limit*limit-previous[0]**2-previous[1]**2)
                fraction = max(0.,min(1.,(-dot+math.sqrt(max(0.,discriminant)))/max(1e-12,length)))
                values[0],values[1] = previous[0]+fraction*dx,previous[1]+fraction*dy
            else:
                values[0] *= limit/speed
                values[1] *= limit/speed
        return tuple(values)

    def normalize(self, command, stopped: bool = False) -> tuple[float, float, float]:
        if not all(math.isfinite(value) for value in command):
            return (0.0, 0.0, 0.0)
        limits = (self.max_x, self.max_y, self.max_yaw)
        values = [max(-limit, min(limit, float(value))) for value, limit in zip(command, limits)]
        if not self.allow_reverse:
            values[0] = max(0.0, values[0])
        values, stopped = apply_command_deadband(values, stopped, self.deadband, self.hysteresis)
        if stopped:
            return (0.0, 0.0, 0.0)
        if self.axis_exclusive:
            values, _ = axis_isolated_command(values, self.yaw_threshold)
        else:
            values = ensure_forward_turning_arc(values, self.yaw_threshold, self.arc_forward, self.arc_yaw)
        # Arc floors must never override actuator/policy bounds.
        values = [max(-limit, min(limit, value)) for value, limit in zip(values, limits)]
        return self.limit_planar(values)

    def rollout(self, pose, command, velocity, horizon: float, dt: float, delay: float = 0.15, command_duration: float = math.inf, next_command=None, filtered_command=None):
        x, y, yaw = pose
        velocity = list(velocity)
        gains = (self.forward_gain, self.lateral_gain, self.yaw_gain)
        requested = self.normalize(command)
        next_requested = self.normalize(next_command) if next_command is not None else (0.0, 0.0, 0.0)
        def response(command):
            lateral = command[1]*self.lateral_response_gain(command[1])
            moving = min(1., math.hypot(command[0]*gains[0],lateral)/.25)
            yaw_gain = gains[2]+moving*((self.walking_yaw_gain or gains[2])-gains[2])
            angular = command[2]*yaw_gain
            if self.response_profile==NATIVE_CONTACT_PROFILE:
                angular = ((1.-moving)*pivot_yaw_rate(command[2],self.payload_active)+
                           moving*command[2]*(self.walking_yaw_gain or gains[2]))
            return (command[0]*gains[0], lateral,
                     angular + command[1]*self.lateral_yaw_gain)
        target = response(requested)
        following = response(next_requested)
        filtered = None if filtered_command is None else list(filtered_command)
        acceleration = (self.accel_x, self.accel_y, self.accel_yaw)
        steps = max(1, math.ceil(horizon / max(0.02, dt)))
        step_dt = horizon / steps
        linear_response = 1.0 - math.exp(-step_dt / max(.01, self.linear_time_constant))
        response_fraction = (linear_response, linear_response,
                             1.0 - math.exp(-step_dt / max(.01, self.yaw_time_constant)))
        trajectory = [(x, y, yaw, 0.0)]
        for step in range(steps):
            # Integrate midpoint velocity; zero target still has a stopping path.
            previous = velocity[:]
            active_dt = max(0.0, (step + 1) * step_dt - max(step * step_dt, delay))
            step_target = target if step * step_dt < command_duration else following
            if filtered is None:
                for axis, limit in enumerate(acceleration):
                    change = max(0.0, limit) * active_dt
                    velocity[axis] += max(-change, min(change, step_target[axis] - velocity[axis]))
            else:
                step_command = requested if step * step_dt < command_duration else next_requested
                immediate_stop = active_dt > 0.0 and not any(step_command)
                previous_filtered = filtered[:]
                for axis, limit in enumerate(acceleration):
                    change = max(0.0, limit) * active_dt
                    if immediate_stop:
                        filtered[axis] = 0.0  # Policy stop bypasses command slew.
                    else:
                        filtered[axis] += max(-change, min(change, step_command[axis] - filtered[axis]))
                filtered = list(self.limit_planar(filtered,previous_filtered))
                physical_target = response(filtered)
                direction = filtered[1] if abs(filtered[1]) > 1e-9 else velocity[1]
                lateral_tau = (self.positive_lateral_time_constant if direction >= 0. and self.positive_lateral_time_constant > 0.
                               else self.linear_time_constant)
                for axis in range(3):
                    fraction = 1.-math.exp(-step_dt/max(.01,lateral_tau)) if axis == 1 else response_fraction[axis]
                    velocity[axis] += (physical_target[axis] - velocity[axis]) * fraction
            vx, vy, wz = [(old + new) * 0.5 for old, new in zip(previous, velocity)]
            heading = yaw + wz * step_dt * 0.5
            x += (math.cos(heading) * vx - math.sin(heading) * vy) * step_dt
            y += (math.sin(heading) * vx + math.cos(heading) * vy) * step_dt
            yaw += wz * step_dt
            trajectory.append((x, y, yaw, (step + 1) * step_dt))
        return trajectory


_PARAMETERS = {
    "max_lin_vel_x": ("max_x", 0.35),
    "max_lin_vel_y": ("max_y", 0.12),
    "max_ang_vel_z": ("max_yaw", 0.35),
    "cmd_accel_x": ("accel_x", 0.22),
    "cmd_accel_y": ("accel_y", 0.16),
    "cmd_accel_yaw": ("accel_yaw", 0.35),
    "cmd_vel_deadband": ("deadband", 0.05),
    "cmd_vel_deadband_hysteresis": ("hysteresis", 0.01),
    "turning_arc_min_forward_speed": ("arc_forward", 0.0),
    "turning_arc_min_yaw_speed": ("arc_yaw", 0.0),
    "axis_exclusive_yaw_threshold": ("yaw_threshold", 0.05),
    "axis_exclusive_cmd_vel": ("axis_exclusive", False),
    "allow_reverse_cmd_vel": ("allow_reverse", True),
    "response_forward_gain": ("forward_gain", 1.0),
    "response_lateral_gain": ("lateral_gain", 1.0),
    "response_yaw_gain": ("yaw_gain", 1.0),
    "response_linear_time_constant": ("linear_time_constant", .25),
    "response_yaw_time_constant": ("yaw_time_constant", .40),
    "max_planar_speed": ("max_planar_speed", math.inf),
    "response_lateral_yaw_gain": ("lateral_yaw_gain", 0.0),
    "response_walking_yaw_gain": ("walking_yaw_gain", 0.0),
    "response_negative_lateral_gain": ("negative_lateral_gain", 0.0),
    "response_positive_lateral_time_constant": ("positive_lateral_time_constant", 0.0),
    "response_profile": ("response_profile", 'legacy'),
}


def load_motion_envelope(node) -> MotionEnvelope:
    values = {}
    for name, (field, default) in _PARAMETERS.items():
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        values[field] = node.get_parameter(name).value
    if values['response_profile'] not in ('legacy',NATIVE_CONTACT_PROFILE):
        raise ValueError('unknown policy response profile')
    return MotionEnvelope(**values)
