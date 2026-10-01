"""ROS integration tests: launch installed nodes, inspect parameters and verify stop output."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

# Keep these tests away from a running vehicle or another workspace's DDS graph.
os.environ['ROS_DOMAIN_ID'] = '117'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy


class RuntimeLaunch(unittest.TestCase):
    def check_profile(self, profile):
        rclpy.init()
        node = rclpy.create_node('obca_launch_probe')
        speeds, routes = [], []
        subscription = node.create_subscription(
            AckermannDriveStamped, '/obca/test_drive', lambda m: speeds.append(m.drive.speed), 10)
        route_subscription = node.create_subscription(
            NavPath, '/obca/raceline', lambda m: routes.append(m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        with tempfile.TemporaryDirectory(prefix='obca-launch-test-') as work:
            with open(Path(work) / 'launch.log', 'w+') as log:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'obca_navigation', f'navigation_{profile}.launch.py',
                     'drive_topic:=/obca/test_drive'], cwd=work, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    expected = {'kinematic_localization', 'obca_planner', 'obca_tracker'}
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        self.assertIsNone(process.poll(), 'launch exited before nodes became ready')
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if expected.issubset(node.get_node_names()) and len(speeds) >= 5:
                            break
                    self.assertTrue(expected.issubset(node.get_node_names()))
                    self.assertGreaterEqual(len(speeds), 5)
                    self.assertTrue(all(v == 0 for v in speeds), 'motion before initial pose/sensors')
                    def parameter(owner, name):
                        client = node.create_client(GetParameters, f'/{owner}/get_parameters')
                        self.assertTrue(client.wait_for_service(timeout_sec=5))
                        future = client.call_async(GetParameters.Request(names=[name]))
                        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                        self.assertTrue(future.done())
                        result = future.result().values[0]
                        node.destroy_client(client)
                        return result
                    self.assertTrue(parameter('kinematic_localization', 'require_initial_pose').bool_value)
                    self.assertTrue(parameter('kinematic_localization', 'slam_mode').bool_value)
                    self.assertFalse(parameter('obca_planner', 'use_sim_time').bool_value)
                    self.assertEqual(parameter('obca_tracker', 'drive_topic').string_value, '/obca/test_drive')
                    if profile == 'sim':
                        self.assertEqual(parameter('obca_planner', 'reference_mode').string_value, 'raceline')
                        self.assertTrue(Path(parameter('obca_planner', 'raceline_file').string_value).is_file())
                        deadline = time.monotonic() + 5
                        while not routes and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        self.assertTrue(routes and len(routes[-1].poses) > 3)
                    else:
                        self.assertEqual(parameter('obca_planner', 'reference_mode').string_value, 'local')
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
                    log.seek(0)
                    output = log.read()
                    print(output)
                    node.destroy_subscription(subscription)
                    node.destroy_subscription(route_subscription)
                    node.destroy_node()
                    rclpy.shutdown()
                self.assertNotIn('process has died', output)

    def test_real_launch(self):
        self.check_profile('real')

    def test_sim_launch(self):
        self.check_profile('sim')


if __name__ == '__main__':
    unittest.main()
