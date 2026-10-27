import numpy as np
import pytest

from unitree_g1_mujoco_bridge.transport_perception import ObservationConsensus,colour_detections


def box(x=1.,colour='red'):
    return dict(id='box_'+colour,kind='box',colour=colour,position=[x,0.,1.01],pixels=30)


def test_distinct_images_confirm_geometry_and_replays_do_not():
    tracker = ObservationConsensus()
    assert not tracker.update([box()],1.)
    assert not tracker.update([box()],1.)
    assert tracker.update([box(1.01)],1.5)['box_red']['confirmed']
    retained = tracker.update([],2.)['box_red']
    assert retained['observed_stamp'] == 1.5


def test_ambiguous_colour_or_moved_object_revokes_prior_estimate():
    tracker = ObservationConsensus()
    tracker.update([box()],1.)
    tracker.update([box()],1.5)
    assert not tracker.update([box(),box(2.)],2.)
    tracker.update([box()],2.5)
    assert tracker.update([box()],3.)
    assert not tracker.update([box(2.)],3.5)
    assert tracker.update([box(2.)],4.)['box_red']['position'][0] == 2.


def test_two_colours_have_independent_confirmation_and_memory_expires():
    tracker = ObservationConsensus()
    tracker.update([box(),box(colour='blue')],1.)
    observed = tracker.update([box(),box(colour='blue')],1.5)
    assert set(observed) == {'box_red','box_blue'}
    assert not tracker.update([],32.)


def test_bad_camera_registration_is_rejected():
    with pytest.raises(ValueError,match='registration'):
        colour_detections(np.zeros((10,10,3)),np.ones((9,10)),np.eye(3),np.zeros(3),np.eye(3))


def test_delayed_or_invalid_frames_cannot_revoke_newer_confirmation():
    tracker = ObservationConsensus()
    tracker.update([box()],1.)
    confirmed = tracker.update([box()],1.5)
    for stamp in (1.,1.5,float('nan'),float('inf')):
        assert tracker.update([box(),box(2.)],stamp) == confirmed
        assert tracker.update([],stamp) == confirmed
    assert tracker.update([box()],2.)['box_red']['observation_count'] == 3


def test_long_acquisition_gap_still_requires_reconfirmation():
    tracker = ObservationConsensus()
    tracker.update([box()],1.)
    assert tracker.update([box()],1.5)
    assert not tracker.update([box()],3.5)
    assert tracker.update([box()],4.)['box_red']['observation_count'] == 2


@pytest.mark.parametrize('identifier,kind', [('box_red','box'),('target_blue','pad')])
def test_support_approach_requires_its_own_distinct_consistent_views(identifier,kind):
    tracker = ObservationConsensus()
    detection = dict(box(),id=identifier,kind=kind)
    tracker.update([detection],1.)
    item = dict(detection,approach_yaw=.2,approach_source='rgbd_support_edge')
    pending = tracker.update([item],1.5)[identifier]
    assert 'approach_yaw' not in pending and 'approach_source' not in pending
    assert tracker.update([item],2.)[identifier]['approach_yaw'] == .2
    item['approach_yaw'] = 1.7
    assert tracker.update([item],2.)[identifier]['approach_yaw']==.2
    changed = tracker.update([item],2.5)[identifier]
    assert 'approach_yaw' not in changed and 'approach_source' not in changed


def test_search_hint_only_directs_view_and_does_not_accept_stale_geometry():
    from types import SimpleNamespace
    from unitree_g1_mujoco_bridge.transport_demo import TransportMission
    turns = []
    viewpoints = []
    mission = SimpleNamespace(search_hints={'box_red':dict(position=[0.,2.,1.],observed_stamp=1.)},
        pose=(0.,0.,0.),seconds=lambda:100.,event=lambda *args,**kwargs:None,
        observe=lambda *args:None,survey_rotate=lambda yaw,identifier:turns.append(yaw),spin=lambda *args:None,
        navigate=lambda pose,**kwargs:viewpoints.append(pose))
    with pytest.raises(RuntimeError,match='did not discover'):
        TransportMission.discover(mission,'box_red')
    assert turns[0] == pytest.approx(np.pi/2)
    assert len(turns)==11
    np.testing.assert_allclose(viewpoints[0],(0.,.6,np.pi/2))
