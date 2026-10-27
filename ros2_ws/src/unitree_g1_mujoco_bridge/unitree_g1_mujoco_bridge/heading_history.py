"""Publication-stamped onboard attitude history; no simulator pose inputs.

DDS source timestamps describe publication, not sensor acquisition. This bounded
approximation removes callback queue delay but does not certify sensor latency.
"""
from collections import deque
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingHistory:
    def __init__(self, max_age=.05, max_gap=.05):
        self.samples = deque(maxlen=1000)
        self.max_age, self.max_gap = max_age, max_gap
        self.last_tick = None
        self.rejections = 0

    def observe(self, stamp, receipt, tick, rpy, gyro):
        values = (stamp, receipt, *rpy, *gyro)
        if (not all(math.isfinite(v) for v in values) or stamp <= 0. or
                not 0. <= receipt-stamp <= self.max_age or
                (self.samples and stamp <= self.samples[-1][0]) or
                (self.last_tick is not None and tick <= self.last_tick) or
                max(abs(rpy[0]), abs(rpy[1])) > .65):
            self.rejections += 1
            return False
        roll, pitch, yaw = rpy
        # Body gyro Z alone is not Euler yaw rate under roll/pitch.
        rate = (math.sin(roll)*gyro[1]+math.cos(roll)*gyro[2])/math.cos(pitch)
        self.samples.append((stamp, wrap(yaw), rate, int(tick), receipt))
        self.last_tick = tick
        while self.samples and stamp-self.samples[0][0] > 2.:
            self.samples.popleft()
        return True

    def propagate(self, pose, source_stamp, now):
        details = dict(clock='dds_source_publication_not_acquisition', valid=False,
                       rejected_samples=self.rejections)
        if not self.samples or not source_stamp <= now:
            return None, details
        latest = self.samples[-1]
        age = now-latest[0]
        details['latest_age_s'] = age
        if not 0. <= age <= self.max_age:
            details['reason'] = 'stale_attitude'
            return None, details
        rows = list(self.samples)
        bracket = next(((a,b) for a,b in zip(rows, rows[1:])
                        if a[0] <= source_stamp <= b[0]), None)
        if bracket is None:
            details['reason'] = 'source_attitude_not_bracketed'
            return None, details
        a,b = bracket
        # Refuse gaps anywhere in the propagation interval, not just its ends.
        if any(right[0]-left[0] > self.max_gap for left,right in zip(rows, rows[1:])
               if right[0] >= source_stamp):
            details['reason'] = 'attitude_gap'
            return None, details
        fraction = (source_stamp-a[0])/(b[0]-a[0])
        source_yaw = a[1]+fraction*wrap(b[1]-a[1])
        delta = wrap(latest[1]-source_yaw)+latest[2]*age
        heading = wrap(pose[2]+delta)
        details.update(valid=True, reason='complete', source_bracket=[list(a),list(b)],
                       latest_sample=list(latest), source_yaw_rad=source_yaw,
                       heading_delta_rad=delta, yaw_rate_rad_s=latest[2],
                       source_stamp=source_stamp, query_stamp=now)
        return (heading, latest[2]), details
