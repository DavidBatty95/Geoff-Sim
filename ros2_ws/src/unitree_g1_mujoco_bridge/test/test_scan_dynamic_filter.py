import math
import unittest

from sensor_msgs.msg import LaserScan

from unitree_g1_mujoco_bridge.scan_dynamic_filter import ScanDynamicFilter


class ScanDynamicFilterTests(unittest.TestCase):
    def test_remove_dynamic_return_clears_nearby_bins_only(self) -> None:
        scan = LaserScan()
        scan.angle_min = -math.pi / 4.0
        scan.angle_max = math.pi / 4.0
        scan.angle_increment = math.pi / 8.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0, 5.0, 2.0, 2.05, 2.1, 5.0, 5.0]

        node = object.__new__(ScanDynamicFilter)
        node.filter_radius = 0.15
        node.range_padding = 0.10
        node.min_filter_range = 0.2

        removed = ScanDynamicFilter._remove_dynamic_return(node, scan, 2.0, 0.0)

        self.assertGreaterEqual(removed, 1)
        self.assertEqual(scan.ranges[0], 5.0)
        self.assertTrue(math.isinf(scan.ranges[3]))
        self.assertEqual(scan.ranges[-1], 5.0)

    def test_remove_dynamic_return_ignores_invalid_ranges(self) -> None:
        scan = LaserScan()
        scan.angle_min = -1.0
        scan.angle_max = 1.0
        scan.angle_increment = 0.5
        scan.range_min = 0.5
        scan.range_max = 5.0
        scan.ranges = [1.0, 1.0, 1.0, 1.0]

        node = object.__new__(ScanDynamicFilter)
        node.filter_radius = 0.2
        node.range_padding = 0.0
        node.min_filter_range = 0.5

        self.assertEqual(ScanDynamicFilter._remove_dynamic_return(node, scan, 0.1, 0.0), 0)
        self.assertEqual(ScanDynamicFilter._remove_dynamic_return(node, scan, 6.0, 0.0), 0)


if __name__ == "__main__":
    unittest.main()


def test_transform_wait_is_latest_only_bounded_and_never_restamps(monkeypatch):
    from types import SimpleNamespace as S
    from rclpy.time import Time
    from tf2_ros import TransformException
    from unitree_social_navigation_msgs.msg import TrackedPersons,TrackedPerson
    import unitree_g1_mujoco_bridge.scan_dynamic_filter as module
    clock = [0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node = object.__new__(ScanDynamicFilter)
    node.carrying,node.payload = False,None
    node.pending_scan,node.last_scan_stamp = None,-math.inf
    node.latest_people = TrackedPersons(persons=[TrackedPerson()])
    node.latest_people.header.frame_id = 'odom'
    node.latest_people.header.stamp = Time(seconds=10.).to_msg()
    node.latest_cloud,node.cloud_timeout = None,.8
    node.get_clock = lambda:S(now=lambda:Time.from_msg(Time(seconds=10.15).to_msg()))
    node.get_logger = lambda:S(warn=lambda *args,**kwargs:None)
    def missing(*args):
        raise TransformException('not yet received')
    node.tf_buffer = S(lookup_transform=missing)
    published = []
    node.pub = S(publish=published.append)
    first = LaserScan()
    first.header.frame_id = 'base_footprint'
    first.header.stamp = Time(seconds=10.).to_msg()
    first.ranges = [2.,3.]
    node._scan_callback(first)
    assert not published
    second = LaserScan()
    second.header.frame_id = 'base_footprint'
    second.header.stamp = Time(seconds=10.1).to_msg()
    second.ranges = [1.,4.]
    clock[0] = .1
    node._scan_callback(second)
    assert not published
    clock[0] = .21
    node._retry_scan()
    assert len(published)==1 and published[0].header.stamp==second.header.stamp
    assert list(published[0].ranges)==[1.,4.] and node.pending_scan is None
    node._scan_callback(first)
    assert len(published)==1
