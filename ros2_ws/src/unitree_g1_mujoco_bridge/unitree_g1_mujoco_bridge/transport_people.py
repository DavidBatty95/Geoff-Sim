"""Age-aware constant-velocity workspace clearance from sensed person tracks."""
import math


def closest_person_distance(position,velocity,robot,age,horizon=2.):
    if not all(math.isfinite(value) for value in (*position,*velocity,*robot,age,horizon)) or age<0. or horizon<0.:
        return 0.  # Invalid measurements cannot establish a clear workspace.
    dx,dy = position[0]+velocity[0]*age-robot[0],position[1]+velocity[1]*age-robot[1]
    vx,vy = velocity
    speed_squared = vx*vx+vy*vy
    t = max(0.,min(horizon,-(dx*vx+dy*vy)/speed_squared)) if speed_squared>1e-12 else 0.
    return math.hypot(dx+vx*t,dy+vy*t)
