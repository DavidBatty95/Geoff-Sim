"""Route-following preference; collision checking remains with the planner."""
import math


def smoothstep(value):
    value = max(0.,min(1.,value))
    return value*value*(3.-2.*value)


def loaded_turn_reference(error,limit=.6):
    """Taper ordinary loaded turns before arrival; emergency commands stay separate."""
    if not math.isfinite(error) or abs(error)<=.12:
        return 0.
    # Native held-arm probes turn at +/- .35; the former .15 floor can
    # equilibrate below the gait's response region and block all translation.
    return math.copysign(min(limit,max(.35,1.2*abs(error))),error)


def forward_reference(local_x, local_y, speed, max_yaw=.35, max_clockwise=None,smooth_loaded=False):
    """Turn toward route lookahead, then walk forward without sustained strafing.

    Blend walking into a pivot over 20--35 degrees instead of switching gait
    at a single noisy bearing. Every resulting arc is checked by the planner.
    """
    if not all(math.isfinite(v) for v in (local_x, local_y, speed, max_yaw)):
        return 0., 0., 0.
    if math.hypot(local_x, local_y) < 1e-6:
        return 0., 0., 0.
    heading = math.atan2(local_y, local_x)
    angular = max(-max_yaw, min(max_yaw, 1.2*heading))
    if max_clockwise is not None:
        # Measured official-policy pivots are directionally asymmetric. Keep
        # clockwise input out of its weak-response region, with a heading dead
        # zone to prevent small-error hunting and unintended constant rotation.
        if not math.isfinite(max_clockwise) or max_clockwise <= 0:
            return 0., 0., 0.
        if abs(heading) <= .12:
            angular = 0.
        elif abs(heading) < math.pi/9:
            # The stepping gait responds much more strongly than a stationary
            # pivot. Do not carry the pivot dead-zone compensation into walking.
            angular = max(-min(.35, max_clockwise), min(.35, max_yaw, 1.2*heading))
        elif heading < 0:
            angular = -min(max_clockwise, .60 + 1.2*abs(heading))
        else:
            angular = min(max_yaw, .15 + 1.2*heading)
    pivot = smoothstep((abs(heading)-math.pi/9)/(math.radians(35)-math.pi/9))
    if max_clockwise is not None and abs(heading)>.12:
        walking = max(-min(.35,max_clockwise),min(.35,max_yaw,1.2*heading))
        turning = (-min(max_clockwise,.60+1.2*abs(heading)) if heading<0.
                   else min(max_yaw,.15+1.2*heading))
        angular = walking+(turning-walking)*pivot
    alignment = max(0., math.cos(heading))
    if smooth_loaded:
        limit = min(.6,max_yaw if heading>=0. else (max_clockwise or max_yaw))
        angular = loaded_turn_reference(heading,limit)
        walking = max(-.35,min(.35,angular))
        angular = walking+(angular-walking)*pivot
    forward = max(0., speed)*alignment*alignment*(1.-pivot)
    return forward, 0., angular


class NominalMotionReference:
    """Stateful nominal command shaping, upstream of all collision rollouts.

    Urgent stops/escapes never use this filter. Alignment has hysteresis and a
    measured-speed interlock: a large new pivot first brakes the walking gait.
    """
    def __init__(self):
        self.command = [0.,0.,0.]
        self.acceleration = [0.,0.,0.]
        self.aligning = False

    def reset(self):
        self.__init__()

    def update(self,target,heading,velocity,dt,loaded=False):
        dt = max(.001,min(.1,dt))
        if abs(heading)>math.radians(35):
            self.aligning = True
        elif abs(heading)<math.radians(18):
            self.aligning = False
        target = list(target)
        if self.aligning:
            target[0] = target[1] = 0.
            # Do not begin a strong pivot while still coasting from travel.
            if math.hypot(*velocity[:2])>.12:
                target[2] = max(-.25,min(.25,target[2]))
        limits = (.7,.6,.65) if loaded else (1.,.8,.9)
        for axis,limit in enumerate(limits):
            delta = target[axis]-self.command[axis]
            desired = max(-limit,min(limit,delta/dt))
            jerk = 3. if axis<2 else 2.5
            self.acceleration[axis] += max(-jerk*dt,min(jerk*dt,desired-self.acceleration[axis]))
            step = self.acceleration[axis]*dt
            if delta*step>0. and abs(step)>=abs(delta):
                self.command[axis] = target[axis]
                self.acceleration[axis] = 0.
            else:
                self.command[axis] += step
        return tuple(self.command)
