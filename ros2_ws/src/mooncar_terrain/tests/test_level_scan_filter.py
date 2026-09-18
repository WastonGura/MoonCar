import math
import pathlib
import sys
import unittest
from collections import deque
from unittest.mock import Mock

from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from level_scan_filter import LevelScanFilter  # noqa: E402


class LevelScanFilterTest(unittest.TestCase):
    def test_only_timestamp_matched_level_scans_reach_slam(self):
        node = Mock(imu=deque(maxlen=50), pending=deque(maxlen=10),
                    published=0, omitted=0)
        node.process_pending.side_effect = lambda: LevelScanFilter.process_pending(node)

        def scan(t):
            msg = LaserScan()
            msg.header.stamp.sec = t
            return msg

        def imu(t, tilt=0):
            msg = Imu()
            msg.header.frame_id = 'imu_link'
            msg.header.stamp.sec = t
            msg.orientation.y = math.sin(math.radians(tilt / 2))
            msg.orientation.w = math.cos(math.radians(tilt / 2))
            return msg

        level, tilted, unsynced = scan(10), scan(11), scan(12)
        LevelScanFilter.on_scan(node, level)
        node.publisher.publish.assert_not_called()
        LevelScanFilter.on_imu(node, imu(10))
        node.publisher.publish.assert_called_once_with(level)
        LevelScanFilter.on_scan(node, tilted)
        LevelScanFilter.on_imu(node, imu(11, 12))
        LevelScanFilter.on_scan(node, unsynced)
        LevelScanFilter.on_imu(node, imu(13))
        self.assertEqual((node.published, node.omitted), (1, 2))
        self.assertEqual(len(node.pending), 0)


if __name__ == '__main__':
    unittest.main()
