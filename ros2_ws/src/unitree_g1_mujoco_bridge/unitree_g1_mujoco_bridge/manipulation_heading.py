"""Bounded IMU/odometry heading reference while the free-base arm moves."""
import math

HEADING_KP = 4.
HEADING_KD = .35

def heading_hold_reference(error,yaw_rate,limit=.6):
    if not all(math.isfinite(v) for v in (error,yaw_rate,limit)) or limit<=0.:
        return 0.
    error = math.atan2(math.sin(error),math.cos(error))
    bound = min(.6,limit)
    return max(-bound,min(bound,HEADING_KP*error-HEADING_KD*yaw_rate))
