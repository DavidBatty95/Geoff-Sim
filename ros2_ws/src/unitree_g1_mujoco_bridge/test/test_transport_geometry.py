from pathlib import Path

import numpy as np
from unitree_g1_mujoco_bridge.transport_demo import navigation_footprint


def test_dynamic_navigation_polygon_contains_the_full_empty_and_loaded_circle():
    for radius in (.48,.64):
        polygon = navigation_footprint(radius)
        xy = np.asarray([(point.x,point.y) for point in polygon.points])
        midpoints = (xy+np.roll(xy,-1,axis=0))/2
        assert len(xy)==16 and np.min(np.linalg.norm(midpoints,axis=1))>=radius

from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics
from unitree_g1_mujoco_bridge.transport_perception import colour_detections,support_approach


def kinematics():
    description = Path(__file__).resolve().parents[2] / 'unitree_g1_description/urdf/g1_29dof_mode_15_brainco_hand.urdf'
    return ArmKinematics(description.read_text())


def test_arm_jacobian_matches_finite_difference_on_robot_urdf():
    kin = kinematics()
    kin.tool_offset = np.array((.12,.01,0.))
    joints = dict(zip(kin.names, np.mean(kin.limits, axis=1)))
    position, _, jacobian, _ = kin.forward(joints)
    for index, name in enumerate(kin.names):
        perturbed = dict(joints)
        perturbed[name] += 1e-6
        measured = (kin.forward(perturbed)[0] - position) / 1e-6
        np.testing.assert_allclose(jacobian[:, index], measured, atol=1e-6)


def test_ik_recovers_reachable_position_without_exceeding_joint_limits():
    kin = kinematics()
    target_state = dict(zip(kin.names, np.mean(kin.limits, axis=1)))
    target = kin.forward(target_state)[0]
    start = {name: value + .08 for name, value in target_state.items()}
    result, error = kin.solve(target, start)
    assert error < .003
    for name, (lower, upper) in zip(kin.names, kin.limits):
        assert lower <= result[name] <= upper


def test_rgbd_box_projection_and_below_table_rejection():
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    rgb[59:70, 45:56, 0] = 220
    depth = np.ones((100, 100))
    intrinsics = np.array(((100., 0., 50.), (0., 100., 50.), (0., 0., 1.)))
    optical = np.array(((0., 0., 1.), (-1., 0., 0.), (0., -1., 0.)))
    detections = colour_detections(rgb, depth, intrinsics, np.array((0., 0., 1.2)), optical)
    assert len(detections) == 1
    assert detections[0]['id'] == 'box_red'
    np.testing.assert_allclose(detections[0]['position'], (1., 0., 1.055), atol=1e-6)
    assert not colour_detections(rgb, depth, intrinsics, np.array((0., 0., .6)), optical)


def test_ik_reaches_failed_native_pickup_without_inward_shoulder_branch():
    kin = kinematics()
    kin.tool_offset = np.array((.12,.01,0.))
    # Recorded target from 20261012T165825Z: native geometry reports upper-arm
    # penetration into the torso even though this is inside the URDF limits.
    blocked = dict(zip(kin.names,(-.00567139,.1525948,.2773698,-.7602669,
                                  -.2090796,.7514498,-.1019206)))
    target,orientation,_,_ = kin.forward(blocked)
    result,error = kin.solve(target,blocked,orientation)
    assert error < .003
    assert result['right_shoulder_roll_joint'] <= -.10
    assert np.linalg.norm(kin.forward(result)[1]-orientation) < .05


def test_trained_carry_posture_is_within_limits_and_lifts_fixture_hand():
    from unitree_g1_mujoco_bridge.payload_postures import PAYLOAD_V1_POSTURES
    kin = kinematics()
    kin.tool_offset = np.array((.12,.01,0.))
    positions = np.array(PAYLOAD_V1_POSTURES[0])
    assert np.all(positions > kin.limits[:,0])
    assert np.all(positions < kin.limits[:,1])
    hand = kin.forward(dict(zip(kin.names,positions)))[0]
    assert .30 < hand[0] < .45 and -.3 < hand[1] < -.1
    assert hand[2] > .32


def test_pickup_approach_uses_measured_support_edge_under_rotation():
    x,y = np.meshgrid(np.linspace(-.55,.55,20),np.linspace(-.5,.5,20))
    points = np.column_stack((x.ravel(),y.ravel(),np.ones(x.size)*.95))
    box = np.array((-.47,0.,1.01))
    for yaw in (0.,.4,1.2):
        R = np.array(((np.cos(yaw),-np.sin(yaw),0.),(np.sin(yaw),np.cos(yaw),0.),(0.,0.,1.)))
        observed = support_approach(points@R.T,box@R.T)
        assert observed is not None
        assert abs(np.arctan2(np.sin(observed-yaw),np.cos(observed-yaw)))<.03
    assert support_approach(points,np.array((2.,0.,1.01))) is None
    assert support_approach(points,np.array((0.,0.,1.01))) is None


def test_destination_side_comes_from_table_edge_not_square_rotation_or_robot_bearing():
    x,y = np.meshgrid(np.linspace(-.55,.55,30),np.linspace(-.5,.5,30))
    points = np.column_stack((x.ravel(),y.ravel(),np.ones(x.size)*.95))
    pad = np.array((-.345,0.,.954))
    for yaw in (0.,.4,1.2,2.7,-2.):
        R = np.array(((np.cos(yaw),-np.sin(yaw),0.),(np.sin(yaw),np.cos(yaw),0.),(0.,0.,1.)))
        shift = np.array((3.2,-1.7,0.))
        observed = support_approach(points@R.T+shift,pad@R.T+shift,max_edge_gap=.30)
        assert observed is not None
        assert abs(np.arctan2(np.sin(observed-yaw),np.cos(observed-yaw)))<.03
    assert support_approach(points,np.array((0.,0.,.954)),max_edge_gap=.30) is None
    assert support_approach(points,np.array((-.345,-.295,.954)),max_edge_gap=.30) is None


def test_rgbd_blue_pad_gets_table_side_and_pad_alone_does_not():
    # Perspective rays intersect a complete horizontal tabletop. The blue pad
    # sits by its near edge, which supplies direction despite square symmetry.
    h,w = 240,320
    K = np.array(((280.,0.,w/2),(0.,280.,h/2),(0.,0.,1.)))
    origin = np.array((0.,0.,1.3))
    pitch = np.deg2rad(25.)
    rotation = np.array(((np.cos(pitch),0.,np.sin(pitch)),(0.,1.,0.),(-np.sin(pitch),0.,np.cos(pitch))))
    optical = rotation@np.array(((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.)))
    v,u = np.indices((h,w))
    rays = np.stack(((u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],np.ones((h,w))),axis=-1)@optical.T
    with np.errstate(divide='ignore',invalid='ignore'):
        distance = (.95-origin[2])/rays[:,:,2]
    points = origin+rays*distance[:,:,None]
    table = (distance>0.)&(points[:,:,0]>=1.2)&(points[:,:,0]<=2.3)&(np.abs(points[:,:,1])<=.5)
    pad = table&(np.abs(points[:,:,0]-1.405)<=.18)&(np.abs(points[:,:,1])<=.18)
    rgb = np.zeros((h,w,3),dtype=np.uint8)
    rgb[table] = (120,100,80)
    rgb[pad] = (20,50,230)
    depth = np.where(table,distance,10.)
    detected = colour_detections(rgb,depth,K,origin,optical)
    target = next(item for item in detected if item['id']=='target_blue')
    assert abs(target['approach_yaw'])<.04
    assert target['approach_source']=='rgbd_support_edge'
    isolated = colour_detections(rgb,np.where(pad,distance,10.),K,origin,optical)
    assert 'approach_yaw' not in next(item for item in isolated if item['id']=='target_blue')
