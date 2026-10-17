import pytest

from audit_native_targets import audit,UnitreeRLLabG1PolicyAdapter,clamp_joint_targets


def test_clipping_audit_separates_motion_from_failed_stop_and_checks_reconstruction():
    adapter = UnitreeRLLabG1PolicyAdapter()
    action = [10.]*29
    clipped = clamp_joint_targets(adapter.action_targets(action))
    data = dict(record_actions=True,actual_command_start_s=5.,actual_command_end_s=7.,
                samples=[dict(time=t,applied_sequence=i,applied_action=action,composed_joint_target=clipped.copy())
                         for i,t in enumerate((5.5,7.5))])
    result = audit(data)
    assert result['clipped_samples']==2 and result['max_reconstruction_error_rad']==0.
    assert result['phases']['moving']['clipped_samples']==1
    assert result['phases']['post_stop']['clipped_samples']==1
    assert result['phases']['moving']['max_target_delta_rad']>0.
    data['samples'][0]['composed_joint_target'][0] += .1
    with pytest.raises(ValueError,match='deployment transform'):
        audit(data)
    data['samples'][0]['composed_joint_target'] = clipped[:10]
    with pytest.raises(ValueError,match='incomplete'):
        audit(data)
