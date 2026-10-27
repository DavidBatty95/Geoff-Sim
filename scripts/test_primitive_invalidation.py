from copy import deepcopy
from dataclasses import asdict

import numpy as np

from analyze_primitive_invalidation import analyze_pair, MotionEnvelope,truth_comparison


def pair():
    old=dict(forecast_epoch=10.,odom_stamp=9.9,tracks_epoch=9.9,prediction_epoch=9.9,
        pose=[0.,0.,0.],velocity=[0.,0.,0.],filtered=[0.,0.,0.],
        program=[[4.5,0.,0.,0.]],envelope=asdict(MotionEnvelope()),
        people=np.tile([[[3.,0.]]],(31,1,1)).tolist(),radii=np.full((31,1),.9).tolist(),
        tracks=[dict(track_id=1,x=3.,y=0.,vx=0.,vy=0.)],goal=[0.,0.],horizon=4.5,dt=.15,delay=.1)
    new=deepcopy(old)
    new.update(forecast_epoch=10.05,odom_stamp=9.95)
    return dict(previous=old,current=new,rejection={'reason':'no_feasible_prefix'})


def test_factorial_replay_identifies_forecast_only_invalidation():
    data=pair()
    data['current']['people']=[[[3.-t,0.]] for t in np.linspace(0.,4.5,31)]
    data['current']['tracks'][0]['vx']=-1.
    result=analyze_pair(data)
    checks=result['counterfactuals']
    assert checks['predicted_robot_old_forecast']['passed']
    assert checks['measured_robot_old_forecast']['passed']
    assert not checks['predicted_robot_new_forecast']['passed']
    assert not checks['measured_robot_new_forecast']['passed']
    assert result['rejecting_tracks']==[1]
    assert result['robot_xy_error_m']==0.


def test_factorial_replay_identifies_robot_state_change_and_rejects_bad_truth_alignment():
    data=pair()
    data['current']['pose']=[2.5,0.,0.]
    result=analyze_pair(data)
    checks=result['counterfactuals']
    assert checks['predicted_robot_old_forecast']['passed']
    assert checks['predicted_robot_new_forecast']['passed']
    assert not checks['measured_robot_old_forecast']['passed']
    assert not checks['measured_robot_new_forecast']['passed']
    assert not truth_comparison(data,[],[{'source_stamp':9.95}],[{'source_stamp':9.97}])['valid']
