import math
from typing import Iterable, Sequence

import numpy as np


def being_overtaken(robot_position,robot_velocity,human_position,human_velocity):
    """Frame-invariant pursuit cue; distinguish being caught from following."""
    if not all(math.isfinite(v) for pair in (robot_position,robot_velocity,human_position,human_velocity) for v in pair):
        return False
    hx,hy = human_velocity
    speed = math.hypot(hx,hy)
    if speed<.25:
        return False
    rx,ry = robot_velocity
    along = (rx*hx+ry*hy)/speed
    across = abs(rx*hy-ry*hx)/speed
    behind = (human_position[0]-robot_position[0])*hx+(human_position[1]-robot_position[1])*hy<0.
    return behind and along>.1 and speed>along+.1 and across<.1


def relative_motion_metrics(
    robot_x: float,
    robot_y: float,
    robot_vx: float,
    robot_vy: float,
    human_x: float,
    human_y: float,
    human_vx: float,
    human_vy: float,
) -> dict[str, float | bool]:
    """Return unconstrained future CPA metrics relative to a person."""
    px = robot_x - human_x
    py = robot_y - human_y
    vx = robot_vx - human_vx
    vy = robot_vy - human_vy
    speed_sq = vx * vx + vy * vy
    radial_dot = px * vx + py * vy
    closing = speed_sq >= 1e-6 and radial_dot < -1e-9
    if not closing:
        return {
            "distance": math.hypot(px, py),
            "tcpa": math.inf,
            "dcpa": math.hypot(px, py),
            "closing": False,
        }
    tcpa = -radial_dot / speed_sq
    return {
        "distance": math.hypot(px, py),
        "tcpa": tcpa,
        "dcpa": math.hypot(px + vx * tcpa, py + vy * tcpa),
        "closing": True,
    }


def ttc_is_relevant(
    metrics: dict[str, float | bool], threshold: float, dcpa_limit: float
) -> bool:
    return (
        bool(metrics["closing"]) and
        0.0 < float(metrics["tcpa"]) <= threshold and
        float(metrics["dcpa"]) <= dcpa_limit
    )


def scan_is_fresh(scan_time: float | None, now: float, max_age: float) -> bool:
    return scan_time is not None and 0.0 <= now - scan_time <= max_age


def trajectory_static_cost(
    trajectory: Iterable[Sequence[float]],
    obstacles: Iterable[tuple[float, float]],
    clearance: float,
    influence_range: float,
) -> tuple[float, bool]:
    obstacle_list = list(obstacles)
    points = list(trajectory)
    if not obstacle_list or not points:
        return 0.0, False
    origin_x, origin_y, origin_yaw = points[0][:3]
    if len(points)*len(obstacle_list) >= 128:
        # Same vertex-distance predicate as below, batched in bounded point
        # blocks. Duplicated scan returns must not multiply the cost or change
        # rejection; only the nearest return matters at each trajectory point.
        obstacles_array = np.asarray(obstacle_list,dtype=float)
        c,s = math.cos(origin_yaw),math.sin(origin_yaw)
        cost = 0.
        for start in range(0,len(points),32):
            positions = np.asarray(points[start:start+32],dtype=float)[:,:2]-(origin_x,origin_y)
            local_x = c*positions[:,0]+s*positions[:,1]
            local_y = -s*positions[:,0]+c*positions[:,1]
            nearest = np.full(len(positions),math.inf)
            for offset in range(0,len(obstacles_array),1024):
                block = obstacles_array[offset:offset+1024]
                nearest = np.minimum(nearest,np.min(np.hypot(local_x[:,None]-block[:,0],
                                                             local_y[:,None]-block[:,1]),axis=1))
            if np.any(nearest<clearance):
                return math.inf,True
            within = nearest<influence_range
            cost += float(np.sum(((influence_range-nearest[within])/influence_range)**2))
        return cost,False
    cost = 0.0
    for point in points:
        x, y = point[:2]
        lx = (
            math.cos(origin_yaw) * (x - origin_x) +
            math.sin(origin_yaw) * (y - origin_y)
        )
        ly = (
            -math.sin(origin_yaw) * (x - origin_x) +
            math.cos(origin_yaw) * (y - origin_y)
        )
        nearest = min(math.hypot(lx - ox, ly - oy) for ox, oy in obstacle_list)
        if nearest < clearance:
            return math.inf, True
        if nearest < influence_range:
            cost += ((influence_range - nearest) / influence_range) ** 2
    return cost, False


def trajectory_increases_clearance(
    vx: float,
    vy: float,
    wz: float,
    risk_points: Iterable[tuple[float, float]],
    horizon: float,
    dt: float,
    minimum_gain: float = 0.03,
    allowed_dip: float = 0.02,
) -> bool:
    """Return true when a local command moves away from every nearby risk set."""
    points = list(risk_points)
    if not points:
        return False
    initial = min(math.hypot(px, py) for px, py in points)
    x = y = heading = 0.0
    minimum = initial
    dt = max(0.01, dt)
    steps = max(1, int(max(dt, horizon) / dt))
    for _ in range(steps):
        x += (math.cos(heading) * vx - math.sin(heading) * vy) * dt
        y += (math.sin(heading) * vx + math.cos(heading) * vy) * dt
        heading += wz * dt
        minimum = min(minimum, min(math.hypot(x - px, y - py) for px, py in points))
    final = min(math.hypot(x - px, y - py) for px, py in points)
    return minimum >= initial - allowed_dip and final >= initial + minimum_gain


def trajectory_makes_target_progress(
    vx: float,
    vy: float,
    wz: float,
    target_x: float,
    target_y: float,
    horizon: float,
    dt: float,
    minimum_progress: float = 0.03,
) -> bool:
    initial = math.hypot(target_x, target_y)
    if initial <= minimum_progress:
        return False
    initial_bearing = abs(math.atan2(target_y, target_x))
    x = y = heading = 0.0
    dt = max(0.01, dt)
    steps = max(1, int(max(dt, horizon) / dt))
    for _ in range(steps):
        x += (math.cos(heading) * vx - math.sin(heading) * vy) * dt
        y += (math.sin(heading) * vx + math.cos(heading) * vy) * dt
        heading += wz * dt
    final_dx = target_x - x
    final_dy = target_y - y
    if math.hypot(final_dx, final_dy) <= initial - minimum_progress:
        return True
    final_bearing = math.atan2(final_dy, final_dx) - heading
    final_bearing = abs((final_bearing + math.pi) % (2.0 * math.pi) - math.pi)
    return final_bearing <= initial_bearing - 0.03


def no_unrequested_forward(candidate_x: float, requested_x: float) -> float:
    if requested_x <= 0.0:
        return min(0.0, candidate_x)
    return candidate_x


def blocked_motion_state(
    current_state: str,
    state_age: float,
    blocked_age: float,
    yield_timeout: float,
    deadlock_timeout: float,
) -> str:
    yielded_long_enough = current_state == "DEADLOCK_RECOVERY" or (
        current_state == "YIELD_TO_HUMAN" and state_age >= yield_timeout
    )
    if yielded_long_enough and blocked_age >= deadlock_timeout:
        return "DEADLOCK_RECOVERY"
    return "YIELD_TO_HUMAN"


def stop_velocity() -> tuple[float, float, float]:
    return 0.0, 0.0, 0.0


def clamp_velocity(
    vx: float,
    vy: float,
    wz: float,
    max_forward: float,
    max_reverse: float,
    max_lateral: float,
    max_angular: float,
) -> tuple[float, float, float]:
    return (
        max(-max_reverse, min(max_forward, vx)),
        max(-max_lateral, min(max_lateral, vy)),
        max(-max_angular, min(max_angular, wz)),
    )
