"""Fault-injection checks for attachment acknowledgements and visual placement."""
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from unitree_g1_mujoco_bridge.transport_demo import TransportMission


class MissionHarness:
    service = TransportMission.service
    held = TransportMission.held
    object_may_be_held = TransportMission.object_may_be_held
    observe = TransportMission.observe
    placement_observed = TransportMission.placement_observed

    def __init__(self,initial='empty'):
        self.attachment_state = initial
        self.phase = 'test'
        self.observations = {}
        self.states = []
        self.now = 12.

    def seconds(self):
        return self.now

    def wait(self,predicate,timeout,message):
        if not predicate():
            raise RuntimeError(message)

    def future(self,result):
        if isinstance(result,Exception):
            raise result
        return result

    def event(self,*args,**kwargs):
        pass

    def _heartbeat(self):
        self.states.append((self.attachment_state,self.object_may_be_held))


@pytest.mark.parametrize('initial,after',[('empty','held'),('held','empty')])
@pytest.mark.parametrize('failure',['timeout','rejected','dispatch_exception'])
def test_dispatched_attachment_failure_preserves_possible_possession(initial,after,failure):
    mission = MissionHarness(initial)

    def dispatch(request):
        assert mission.attachment_state == 'uncertain'
        assert mission.object_may_be_held
        if failure == 'dispatch_exception':
            raise RuntimeError('transport failed during dispatch')
        if failure == 'timeout':
            return TimeoutError('acknowledgement lost')
        return SimpleNamespace(success=False,message='actuator rejected request')

    client = SimpleNamespace(service_is_ready=lambda:True,call_async=dispatch)
    with pytest.raises((RuntimeError,TimeoutError)):
        mission.service(client,object(),attachment_after=after)
    assert mission.attachment_state == 'uncertain'
    assert mission.object_may_be_held
    assert not mission.held
    assert mission.states == [('uncertain',True)]


@pytest.mark.parametrize('initial,after',[('empty','held'),('held','empty')])
def test_unavailable_service_preserves_prior_attachment_state(initial,after):
    mission = MissionHarness(initial)
    client = SimpleNamespace(service_is_ready=lambda:False)
    with pytest.raises(RuntimeError,match='unavailable'):
        mission.service(client,object(),attachment_after=after)
    assert mission.attachment_state == initial
    assert not mission.states


def test_acknowledged_grasp_and_release_resolve_uncertainty():
    mission = MissionHarness()
    client = SimpleNamespace(service_is_ready=lambda:True,
                             call_async=lambda request:SimpleNamespace(success=True))
    mission.service(client,object(),attachment_after='held')
    assert mission.held and mission.object_may_be_held
    mission.service(client,object(),attachment_after='empty')
    assert not mission.held and not mission.object_may_be_held
    assert mission.states == [('uncertain',True),('held',True),
                              ('uncertain',True),('empty',False)]


def test_authority_service_does_not_change_attachment_state():
    mission = MissionHarness('held')
    client = SimpleNamespace(service_is_ready=lambda:True,
                             call_async=lambda request:SimpleNamespace(success=True))
    mission.service(client,object())
    assert mission.held
    assert not mission.states


@pytest.mark.parametrize('stamp,position,expected',[
    (9.9,(2.,0.,1.009),False),  # Fresh but acquired before acknowledgement.
    (10.,(2.,0.,1.009),False),  # Equal timestamp is not post-release evidence.
    (11.,(2.,0.,1.009),True),
    (6.,(2.,0.,1.009),False),   # Expired observation.
    (13.,(2.,0.,1.009),False),  # Future timestamp.
    (11.,(2.3,0.,1.009),False), # Outside destination.
    (11.,(2.,0.,1.2),False),   # Still elevated.
])
def test_placement_requires_fresh_post_acknowledgement_geometry(stamp,position,expected):
    mission = MissionHarness()
    mission.observations['box_red'] = dict(observed_stamp=stamp,position=position,confirmed=True)
    assert mission.placement_observed('box_red',np.array((2.,0.,.954)),10.) is expected


def test_missing_box_cannot_verify_placement():
    assert not MissionHarness().placement_observed('box_red',np.array((2.,0.,.954)),10.)


def test_unconfirmed_image_candidate_is_not_a_manipulation_target():
    mission = MissionHarness()
    mission.observations['box_red'] = dict(observed_stamp=11.,position=(2.,0.,1.009),confirmed=False)
    assert mission.observe('box_red') is None


def test_uncertain_attachment_publishes_conservative_carrying_and_payload():
    mission = MissionHarness('uncertain')
    messages = {}
    for name in ('active','mode','carry','status','payload'):
        setattr(mission,name+'_pub',SimpleNamespace(
            publish=lambda message,key=name:messages.__setitem__(key,message)))
    mission.active,mission.mode = False,'navigate'
    mission.navigation_empty_radius,mission.navigation_radius = .48,None
    mission.last_footprint_time = -float('inf')
    mission.footprint_pubs = [SimpleNamespace(publish=lambda msg:messages.__setitem__('footprint',msg))]
    mission.started = time.monotonic()
    mission.kin = SimpleNamespace(forward=lambda joints:(np.zeros(3),np.eye(3),None,None))
    mission.joints,mission.rpy,mission.offset = {},(0.,0.,0.),np.array((.1,0.,0.))
    from builtin_interfaces.msg import Time
    mission.get_clock = lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time()))
    TransportMission._heartbeat(mission)
    assert mission.navigation_radius == .64 and len(messages['footprint'].points)==16
    assert messages['carry'].data
    status = json.loads(messages['status'].data)
    assert status['attachment_state'] == 'uncertain'
    assert status['object_may_be_held'] and not status['held']
    assert messages['payload'].header.frame_id == 'base_footprint'
    assert messages['payload'].point.x == pytest.approx(.1)
