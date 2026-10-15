from record_transport_preview import mujoco_windows
import json
import pytest
from export_transport_gifs import export_clip


def test_existing_mujoco_viewer_is_not_the_new_transport_window():
    existing = mujoco_windows('  0x100 "MuJoCo : Dougie"\n  0x200 "Other app"')
    later = mujoco_windows('  0x100 "MuJoCo : Dougie"\n  0x300 "MuJoCo : G1"')
    candidates = {key:value for key,value in later.items() if key not in existing}
    assert list(candidates)==['0x300']
    assert not mujoco_windows('no X11 windows')


def test_export_rejects_footage_marked_as_wrong_window(tmp_path):
    (tmp_path/'capture.json').write_text(json.dumps(dict(media_valid=False,media_invalid_reason='wrong viewer')))
    with pytest.raises(ValueError,match='wrong viewer'):
        export_clip(tmp_path,tmp_path/'must-not-exist.gif','clip',None,(0,0,10,10))
    assert not (tmp_path/'must-not-exist.gif').exists()
