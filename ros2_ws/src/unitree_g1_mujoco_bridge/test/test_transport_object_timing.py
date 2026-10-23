from types import SimpleNamespace

from vision_msgs.msg import Detection3D,Detection3DArray,ObjectHypothesisWithPose

from unitree_g1_mujoco_bridge.transport_demo import TransportEvaluator
from unitree_g1_mujoco_bridge.transport_health import EvaluationHealth


def test_replayed_objects_cannot_extend_placement_dwell_or_evaluation_health():
    clock = [10.1]
    node = SimpleNamespace(objects={},shapes={},initial={},history={},orientation_history={},drops=set(),
        object_timestamp_sources=set(),health=EvaluationHealth({'box_red'},clock=lambda:clock[0]),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9))))
    detection = Detection3D()
    detection.id = 'native_acquisition_v2:box_red'
    detection.header.stamp.sec = 10
    detection.bbox.center.position.z = 1.
    detection.bbox.center.orientation.w = 1.
    detection.bbox.size.x = detection.bbox.size.y = detection.bbox.size.z = .11
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = 'box_red'
    detection.results.append(hypothesis)
    msg = Detection3DArray(detections=[detection])
    TransportEvaluator._objects(node,msg)
    node.health.start()
    for _ in range(20):
        TransportEvaluator._objects(node,msg)
    assert len(node.history['box_red']) == 1
    assert len(node.orientation_history['box_red']) == 1
    assert node.health.updated['box_red'] == 10.1
    clock[0] = 12.
    TransportEvaluator._objects(node,msg)
    assert len(node.history['box_red']) == 1
    assert 'box_red: native acquisition age invalid' in node.health.failures
    assert not node.health.report()['valid']
