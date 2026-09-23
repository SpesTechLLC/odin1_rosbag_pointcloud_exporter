"""Integration checks with a tiny MCAP whose device clock differs from wall time."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message

REPO = Path(__file__).resolve().parents[1]
START_NS = 1790136489000000000  # 2026-09-23 13:08:09 JST


class CameraPathTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / 'bag'
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(self.bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        writer.create_topic(rosbag2_py.TopicMetadata(
            name='/odin1/odometry', type='nav_msgs/msg/Odometry',
            serialization_format='cdr'))
        for i in range(5):
            msg = Odometry()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'odin1_base_link'
            msg.header.stamp.sec = 100 + i  # intentionally unrelated to wall time
            msg.pose.pose.position.x = float(i)
            msg.pose.pose.position.z = float(i) * 0.1
            msg.pose.pose.orientation.w = 1.0
            writer.write('/odin1/odometry', serialize_message(msg), START_NS + i * 10**9)
        del writer

    def run_path(self, name, *args, success=True):
        output = self.root / (name + '.yaml')
        result = subprocess.run(
            ['bash', str(REPO / 'make_camera_path_from_bag.sh'), str(self.bag),
             '--output', str(output), '--interval', '1', *args],
            text=True, capture_output=True,
            env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return yaml.safe_load(output.read_text()), output
        self.assertNotEqual(result.returncode, 0)
        return result, output

    def test_record_clock_range_interpolation_and_camera_axes(self):
        doc, output = self.run_path('window', '--start', '0.5', '--end', '3.5',
                                    '--duration', '3801')
        self.assertEqual(doc['durationSeconds'], 3801)
        self.assertEqual(len(doc['waypoints']), 4)
        self.assertEqual(len({w['id'] for w in doc['waypoints']}), 4)
        positions = np.array([w['position'] for w in doc['waypoints']])
        np.testing.assert_allclose(positions[:, 0], [0.5, 1.5, 2.5, 3.5])
        np.testing.assert_allclose(positions[:, 2], [0.05, 0.15, 0.25, 0.35])
        for waypoint in doc['waypoints']:
            forward = np.array(waypoint['target']) - waypoint['position']
            forward /= np.linalg.norm(forward)
            q = np.array(waypoint['quaternion'])
            self.assertAlmostEqual(np.linalg.norm(q), 1)
            np.testing.assert_allclose(Rotation.from_quat(q).apply([0, 0, -1]), forward)
            self.assertGreater(Rotation.from_quat(q).apply([0, 1, 0])[2], 0)
        source = np.genfromtxt(output.with_suffix('.source.csv'), delimiter=',', names=True)
        self.assertGreater(source['timestamp'][0], 10**9)
        self.assertLess(source['header_timestamp'][-1], 200)

    def test_clock_iso_and_cached_elapsed_agree(self):
        wall, path = self.run_path('wall', '--start', '13:08:09.5', '--end', '13:08:12.5')
        cached = str(path.with_suffix('.source.csv'))
        iso, _ = self.run_path('iso', '--cached-csv', cached,
                              '--start', '2026-09-23T13:08:09.5+09:00',
                              '--end', '2026-09-23T13:08:12.5+09:00')
        elapsed, _ = self.run_path('elapsed', '--cached-csv', cached,
                                  '--start', '0.5', '--end', '3.5')
        for other in [iso, elapsed]:
            self.assertEqual(other['durationSeconds'], 3)
            for a, b in zip(wall['waypoints'], other['waypoints']):
                for key in ['position', 'target', 'quaternion']:
                    np.testing.assert_allclose(a[key], b[key], atol=1e-10)

    def test_bad_range_and_overwrite_are_rejected(self):
        _, output = self.run_path('bad', '--start', '3', '--end', '1', success=False)
        self.assertFalse(output.exists())
        self.run_path('existing')
        original = (self.root / 'existing.yaml').read_bytes()
        self.run_path('existing', success=False)
        self.assertEqual((self.root / 'existing.yaml').read_bytes(), original)

    def test_cached_interior_boundary_must_be_covered(self):
        _, path = self.run_path('source')
        cache = path.with_suffix('.source.csv')
        data = np.loadtxt(cache, delimiter=',', skiprows=1)[1:]
        np.savetxt(cache, data, delimiter=',', comments='',
                   header='timestamp,x,y,z,qx,qy,qz,qw,header_timestamp')
        result, output = self.run_path('uncovered', '--cached-csv', str(cache),
                                       '--start', '0.8', '--end', '3', success=False)
        self.assertIn('not covered', result.stderr)
        self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
