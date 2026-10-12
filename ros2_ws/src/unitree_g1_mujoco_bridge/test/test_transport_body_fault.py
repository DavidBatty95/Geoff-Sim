import time
from types import SimpleNamespace as S

import pytest

from unitree_g1_mujoco_bridge.transport_demo import TransportMission
from unitree_g1_mujoco_bridge.localization_recovery import BodyInstabilityError


def message(stamp,roll):
    sec = int(stamp)
    return S(header=S(stamp=S(sec=sec,nanosec=round((stamp-sec)*1e9))),vector=S(x=roll,y=0.,z=0.))


def mission():
    node = S(active=False,pose=None,rpy=(0.,0.,0.),started=time.monotonic(),args=S(duration=100.),
             attachment_state='held',events=[],seconds=lambda:10.21)
    node.stop_intent = lambda:setattr(node,'active',False)
    node.event = lambda *args,**kwargs:node.events.append((args,kwargs))
    return node


def test_sustained_fresh_tilt_is_terminal_and_preserves_possession():
    node = mission()
    for stamp in (10.,10.1,10.21):
        TransportMission._body_orientation(node,message(stamp,.9))
    with pytest.raises(BodyInstabilityError):
        TransportMission.spin(node,0.)
    assert node.attachment_state=='held' and not node.active
    assert node.events[0][0]==('body_instability',)


def test_single_sample_replay_gap_or_stale_tilt_cannot_establish_sustained_fault():
    node = mission()
    TransportMission._body_orientation(node,message(10.,.9))
    TransportMission._body_orientation(node,message(10.,0.))
    assert node.rpy[0]==.9
    TransportMission.spin(node,0.)
    TransportMission._body_orientation(node,message(10.5,.9))
    assert node.unstable_since==10.5
    TransportMission._body_orientation(node,message(10.6,0.))
    assert node.unstable_since is None
    TransportMission._body_orientation(node,message(10.7,.9))
    TransportMission._body_orientation(node,message(10.91,.9))
    node.seconds = lambda:12.
    TransportMission.spin(node,0.)
    assert not node.events


def test_body_fault_terminates_navigation_without_localization_retry():
    from contextlib import contextmanager
    from geometry_msgs.msg import PoseStamped
    calls = []
    @contextmanager
    def action(*args):
        calls.append('dispatched')
        try:
            yield S(result=S(done=lambda:False))
        finally:
            calls.append('terminated')
    def wait(predicate,*args):
        if not predicate():
            raise BodyInstabilityError('sustained measured body instability')
    node = S(goal=PoseStamped(),active=True,pose=(0.,0.,0.),pose_stamp=10.,seconds=lambda:10.,
        nav=S(server_is_ready=lambda:True),set_goal=lambda *args:None,_heartbeat=lambda:None,
        wait=wait,action=action,attachment_state='held')
    with pytest.raises(BodyInstabilityError):
        TransportMission.navigate(node,(3.,0.,0.))
    assert calls==['dispatched','terminated'] and not node.active
    assert node.attachment_state=='held'
