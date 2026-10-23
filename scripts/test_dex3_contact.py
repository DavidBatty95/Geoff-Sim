"""Run separately in the pinned native MuJoCo environment."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from dex3_hand_model import ROOT,build_robot,joint_names,SCENES,PACKAGE
from measure_dex3_contact_grasp import run
from unitree_g1_mujoco_bridge.arm_kinematics import ArmKinematics


@pytest.mark.parametrize('side,x,y,radius',[
    ('right',.135,.04,.02),('left',.135,.04,.02),
    ('right',.132,.038,.019),('right',.138,.042,.021)])
def test_real_contacts_lift_hold_and_release_without_object_constraint(side,x,y,radius):
    result = run(side,x,y,radius,.08)
    assert result['success'],{k:v for k,v in result.items() if k!='observations'}
    assert result['object_constraints']==0 and not result['object_external_force']
    assert result['opposed_contact'] and result['minimum_held_lift_m']>.1
    assert result['released']


def test_closing_empty_fingers_cannot_claim_a_grasp():
    result = run('right',.35,.2,.02,.08)
    assert not result['success'] and not result['opposed_contact']
    assert result['minimum_held_lift_m']<.01


@pytest.mark.parametrize('side',['right','left'])
def test_hand_retains_grasp_during_extended_wrist_sway(side):
    result = run(side,.135,.04,.02,.08,hold_seconds=40.,sway_m=.01)
    assert result['success'],{k:v for k,v in result.items() if k!='observations'}
    assert result['object_constraints']==0 and not result['object_external_force']


@pytest.mark.parametrize('side',['right','left'])
def test_measured_slow_opening_places_upright_on_box_support(side):
    result = run(side,.135,.04,.02,.08,support='box',release_gap=.03,
                 release_rate=.3,release_from_measured=True)
    assert result['success'] and result['final_upright_cosine']>.98
    assert np.linalg.norm(np.array(result['final_position'])[:2]-[.135,.04 if side=='right' else -.04])<.06


def test_raised_grasp_retains_contact_and_releases_upright():
    result = run('right',.135,.04,.02,.08,support='box',grasp_height_offset=.02,
                 hold_seconds=40.,sway_m=.01,release_rate=.3,release_from_measured=True)
    assert result['success'] and result['opposed_contact'] and result['upright_release']
    assert result['object_constraints']==0 and not result['object_external_force']


def test_tool_referenced_rotation_preserves_grasp_and_exercises_real_yaw():
    result = run('right',.136,.044,.02,.08,support='box',grasp_height_offset=.015,
        hold_seconds=40.,sway_m=.01,turns=True,wrist_pitch=.06,pitch_about_tool=True,
        lift_retract=.1,release_rate=.3,release_from_measured=True)
    assert result['success'] and result['opposed_contact']
    assert result['max_hand_table_force_before_lift_n']<.05
    assert max(row['wrist_angles_rad'][0] for row in result['observations'])>3.
    held = [row for row in result['observations'] if 5.2<=row['time']<=44.9]
    assert max(abs(row['tool_position'][2]-1.165) for row in held)<.006
    assert result['object_constraints']==0 and not result['object_external_force']


def test_body_actuator_contract_survives_added_hand_dofs():
    model = mujoco.MjModel.from_xml_path(str(SCENES/'g1_29dof_dex3.xml'))
    assert model.nu==29 and model.nq==50 and model.nv==49
    assert model.neq==0
    for side in ('left','right'):
        for name in joint_names(side):
            joint = model.joint(name).id
            assert not np.any(model.actuator_trnid[:,0]==joint)
    description = ET.parse(PACKAGE/'urdf/g1_29dof_dex3.urdf').getroot()
    assert all(description.find(f"joint[@name='{name}']") is not None for side in ('left','right') for name in joint_names(side))
    links = {link.get('name') for link in description.findall('link')}
    semantics = ET.parse(ROOT/'ros2_ws/src/g1_moveit_config/config/g1_dex3.srdf').getroot()
    for chain in semantics.findall('group/chain'):
        assert chain.get('base_link') in links and chain.get('tip_link') in links
        assert chain.get('tip_link').endswith('wrist_yaw_link')
    for collision in semantics.findall('disable_collisions'):
        assert collision.get('link1') in links and collision.get('link2') in links


@pytest.mark.parametrize('side',['right','left'])
def test_urdf_tool_kinematics_match_native_robot_with_measured_waist(side):
    offset = (.135,.04 if side=='right' else -.04,0.)
    kin = ArmKinematics((PACKAGE/'urdf/g1_29dof_dex3.urdf').read_text(),side=side,tool_offset=offset)
    model = mujoco.MjModel.from_xml_path(str(SCENES/'g1_29dof_dex3.xml'))
    data = mujoco.MjData(model)
    q = dict(zip(kin.names,[-.5,-.6 if side=='right' else .6,.1,.2,.7,.3,-.2]))
    q.update(waist_yaw_joint=.1,waist_pitch_joint=-.05,waist_roll_joint=.03)
    for name,value in q.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model,data)
    wrist,pelvis = model.body(side+'_wrist_yaw_link').id,model.body('pelvis').id
    native = data.xmat[pelvis].reshape(3,3).T@(data.xpos[wrist]+
        data.xmat[wrist].reshape(3,3)@np.asarray(offset)-data.xpos[pelvis])
    assert np.linalg.norm(native-kin.forward(q)[0]) < .001
