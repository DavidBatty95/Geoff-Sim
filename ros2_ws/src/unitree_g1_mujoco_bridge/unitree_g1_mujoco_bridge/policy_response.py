"""Policy-identified native response interpolation, shared by control and rollout.

Cycle-05 knots are six-second MuJoCo 3.3.6 probes at 40 ms delay (2026-10-12).
Held-arm knots are prescribed posture without a payload, not a loaded guarantee.
These estimates replace a constant gain; live qualification remains necessary.
"""
import math

NATIVE_CONTACT_POLICY_SHA256 = '4582897893b9730781b2e2ddd682eb312479184b636582efb4f67d59fc338c6c'
NATIVE_CONTACT_PROFILE = 'native_contact_cycle05'
COMMANDS = (0.,.35,.6,.9)
RATES = {
    (False,False):(0.,.048712,.464810,.609889),
    (False,True):(0.,.161848,.488955,.766074),
    (True,False):(0.,.095017,.520587,.680709),
    (True,True):(0.,.179795,.545683,.845198),
}


def interpolate(value,xs,ys):
    value = max(xs[0],min(xs[-1],value))
    for index in range(1,len(xs)):
        if value<=xs[index]:
            fraction = (value-xs[index-1])/(xs[index]-xs[index-1])
            return ys[index-1]+fraction*(ys[index]-ys[index-1])
    return ys[-1]


def pivot_yaw_rate(command,loaded=False):
    return math.copysign(interpolate(abs(command),COMMANDS,RATES[bool(loaded),command<0.]),command)


def calibrated_turn_reference(error,measured_rate,loaded=False):
    if not all(map(math.isfinite,(error,measured_rate))) or abs(error)<=.10:
        return 0.
    # Bounded rate feedback around an inverse response curve. No integral state
    # can wind up against collision intervention or the command envelope.
    desired = math.copysign(min(.45,1.4*max(0.,abs(error)-.06)),error)
    corrected = desired+.2*(desired-measured_rate)
    corrected = math.copysign(min(.45,max(0.,corrected*math.copysign(1.,error))),error)
    rates = RATES[bool(loaded),corrected<0.]
    return math.copysign(interpolate(abs(corrected),rates,COMMANDS),corrected)
