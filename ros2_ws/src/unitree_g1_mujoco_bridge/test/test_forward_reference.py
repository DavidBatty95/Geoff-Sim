import math

from unitree_g1_mujoco_bridge.forward_reference import forward_reference,loaded_turn_reference,NominalMotionReference


def test_forward_goal_uses_full_speed():
    assert forward_reference(3, 0, .65) == (.65, 0., 0.)


def test_side_and_rear_goals_turn_before_translation():
    for x, y in ((0, 3), (-3, 0), (-3, -2)):
        vx, vy, yaw = forward_reference(x, y, .65)
        assert vx == vy == 0
        assert 0 < abs(yaw) <= .35
        assert math.copysign(1, yaw) == math.copysign(1, math.atan2(y, x))


def test_forward_arc_and_bad_input():
    vx, vy, yaw = forward_reference(3, 1, .65)
    assert 0 < vx < .65 and vy == 0 and yaw > 0
    assert forward_reference(float('nan'), 1, .65) == (0., 0., 0.)


def test_directional_compensation_respects_calibrated_limits_and_stops_aligned():
    assert forward_reference(0, -2, .6, .6, .9)[2] == -.9
    assert forward_reference(0, 2, .6, .6, .9)[2] == .6
    assert forward_reference(2, .1, .6, .6, .9)[2] == 0.
    assert forward_reference(2, -.1, .6, .6, .9)[2] == 0.
    vx, vy, yaw = forward_reference(3, -1, .6, .6, .9)
    assert vx > 0 and vy == 0 and -.35 <= yaw < 0


def test_loaded_turn_tapers_in_both_directions_without_clockwise_floor_jump():
    for sign in (-1.,1.):
        assert loaded_turn_reference(sign*1.) == sign*.6
        assert abs(loaded_turn_reference(sign*.2)) == .35
        assert loaded_turn_reference(sign*.1) == 0.
        command = forward_reference(0.,sign*2.,.6,.6,.9,smooth_loaded=True)
        assert command == (0.,0.,sign*.6)


def test_alignment_boundary_has_no_command_jump():
    for sign in (-1.,1.):
        commands = [forward_reference(math.cos(a),math.sin(a),.45,.6,.9)
                    for a in (sign*math.radians(19.9),sign*math.radians(20.1))]
        assert max(abs(a-b) for a,b in zip(*commands))<.01


def test_large_pivot_brakes_before_yaw_and_hysteresis_rejects_bearing_noise():
    reference = NominalMotionReference()
    for _ in range(40):
        command = reference.update((.45,0.,0.),0.,(.45,0.,0.),.05)
    assert abs(command[0]-.45)<.01
    for _ in range(40):
        command = reference.update((0.,0.,-.9),-1.,(.4,0.,0.),.05)
    assert abs(command[0])<.01 and abs(command[2])<=.251
    for degrees in (25,32,22,30):
        reference.update((.3,0.,.5),math.radians(degrees),(0.,0.,0.),.05)
        assert reference.aligning
    reference.update((.3,0.,.2),math.radians(15),(0.,0.,0.),.05)
    assert not reference.aligning
