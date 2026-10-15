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


def closest_person_corridor_distance(position,velocity,start,end,age,horizon=2.):
    """Conservative distance between predicted walking and approach segments.

    This is spatial admission, not a synchronized robot rollout: every point of
    the short docking corridor must stay clear of the person's predicted segment.
    """
    if (not all(math.isfinite(v) for v in (*position,*velocity,*start,*end,age,horizon)) or
            age<0. or horizon<0.):
        return 0.
    p = (position[0]+age*velocity[0],position[1]+age*velocity[1])
    q = (p[0]+horizon*velocity[0],p[1]+horizon*velocity[1])
    def subtract(a,b):
        return a[0]-b[0],a[1]-b[1]
    def cross(a,b):
        return a[0]*b[1]-a[1]*b[0]
    first,second = subtract(q,p),subtract(end,start)
    determinant = cross(first,second)
    if abs(determinant)>1e-12:
        offset = subtract(start,p)
        t,u = cross(offset,second)/determinant,cross(offset,first)/determinant
        if 0.<=t<=1. and 0.<=u<=1.:
            return 0.
    def point_segment(point,a,b):
        delta = subtract(b,a)
        length = delta[0]**2+delta[1]**2
        fraction = max(0.,min(1.,((point[0]-a[0])*delta[0]+(point[1]-a[1])*delta[1])/length)) if length>1e-12 else 0.
        return math.hypot(point[0]-a[0]-fraction*delta[0],point[1]-a[1]-fraction*delta[1])
    return min(point_segment(p,start,end),point_segment(q,start,end),
               point_segment(start,p,q),point_segment(end,p,q))
