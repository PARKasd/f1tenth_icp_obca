"""ROS integration tests: launch installed nodes, inspect parameters and verify stop output."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import yaml

# Keep these tests away from a running vehicle or another workspace's DDS graph.
os.environ['ROS_DOMAIN_ID'] = '117'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path as NavPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from f110_msgs.msg import Wpnt, WpntArray
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy


class RuntimeLaunch(unittest.TestCase):
    def test_short_local_path_moves_then_stops(self):
        """Reproduce the live 11.6 cm path vs 12 cm arrival tolerance conflict."""
        rclpy.init()
        node = rclpy.create_node('obca_short_path_probe')
        speeds = []
        node.create_subscription(AckermannDriveStamped, '/obca/test_drive',
                                 lambda m: speeds.append(m.drive.speed), 10)
        initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        pose = node.create_publisher(Odometry, '/pf/pose/odom', 10)
        scan = node.create_publisher(LaserScan, '/scan', 10)
        diag = node.create_publisher(DiagnosticArray, '/kinematic_localization/diagnostics', 10)
        path = node.create_publisher(WpntArray, '/obca/waypoints', 10)
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(
                ['ros2', 'run', 'obca_navigation', 'tracker_node', '--ros-args',
                 '-p', 'drive_topic:=/obca/test_drive'], stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 10
                while initial.get_subscription_count() == 0 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                self.assertGreater(initial.get_subscription_count(), 0)
                reset = PoseWithCovarianceStamped()
                reset.header.frame_id = 'map'
                reset.pose.pose.orientation.w = 1.0
                initial.publish(reset)
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)

                def feed(x):
                    speeds.clear()
                    deadline = time.monotonic() + 0.7
                    while time.monotonic() < deadline:
                        stamp = node.get_clock().now().to_msg()
                        odom = Odometry()
                        odom.header.frame_id = 'map'
                        odom.header.stamp = stamp
                        odom.pose.pose.orientation.w = 1.0
                        odom.pose.pose.position.x = x
                        pose.publish(odom)
                        laser = LaserScan()
                        laser.header.stamp = stamp
                        laser.range_min, laser.range_max, laser.ranges = 0.01, 10.0, [2.0]
                        scan.publish(laser)
                        health = DiagnosticArray()
                        health.header.stamp = stamp
                        health.status = [DiagnosticStatus(values=[KeyValue(key=k, value=v) for k, v in
                            [('inlier_ratio', '1.0'), ('residual_rms', '0.01'),
                             ('dead_reckoning_sec', '0'), ('converged', 'true'),
                             ('gate_rejected', 'false'), ('pose_impermissible', 'false')]])]
                        diag.publish(health)
                        waypoints = WpntArray()
                        waypoints.header.frame_id, waypoints.header.stamp = 'map', stamp
                        waypoints.wpnts = [Wpnt(x_m=d, s_m=d, vx_mps=v) for d, v in
                                           [(0.0, 0.0), (0.05, 0.2), (0.116, 0.0)]]
                        path.publish(waypoints)
                        rclpy.spin_once(node, timeout_sec=0.02)
                    return speeds[-5:]

                self.assertTrue(any(v > 0 for v in feed(0.0)), 'short valid path never moves')
                self.assertTrue(all(v == 0 for v in feed(0.112)), 'path end does not stop')
                speeds.clear()
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                self.assertTrue(speeds and all(v == 0 for v in speeds[-5:]), 'stale path permits motion')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                node.destroy_node()
                rclpy.shutdown()

    def check_profile(self, profile, local=False):
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
                     'drive_topic:=/obca/test_drive'] + (['reference_mode:=local'] if local else []), cwd=work, stdout=log,
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
                    package = Path(__file__).parents[1]
                    common = yaml.safe_load((package / 'config' / 'navigation.yaml').read_text())['/**']['ros__parameters']
                    profile_values = yaml.safe_load((package / 'config' / f'{profile}.yaml').read_text())
                    goals = dict(common)
                    goals.update(profile_values.get('/**', {}).get('ros__parameters', {}))
                    goals.update(profile_values.get('obca_planner', {}).get('ros__parameters', {}))
                    for name in ('goal_progress_weight', 'goal_route_clearance_weight', 'goal_turn_weight',
                                 'goal_continuation_weight', 'goal_clearance_target', 'goal_continuation_distance'):
                        self.assertEqual(parameter('obca_planner', name).double_value, goals[name])
                    self.assertEqual(parameter('obca_planner', 'retain_observations').bool_value,
                                     goals['retain_observations'])
                    if profile == 'sim':
                        profile_params = yaml.safe_load((Path(__file__).parents[1] / 'config' / 'sim.yaml').read_text())
                        for owner, names in [('obca_planner', ('reference_wall_weight', 'goal_clearance_weight',
                                                             'position_weight', 'max_speed', 'solve_seconds',
                                                             'reference_distance', 'planning_period', 'dt',
                                                             'max_accel', 'max_steering_rate')),
                                             ('obca_tracker', ('lookahead', 'max_speed', 'dt',
                                                               'max_accel', 'max_steering_rate'))]:
                            for name in names:
                                self.assertEqual(parameter(owner, name).double_value,
                                                 profile_params[owner]['ros__parameters'].get(name,
                                                     profile_params['/**']['ros__parameters'].get(name)))
                        self.assertEqual(parameter('obca_planner', 'horizon').integer_value,
                                         profile_params['obca_planner']['ros__parameters']['horizon'])
                    if profile == 'sim' and not local:
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

    def test_sim_local_launch(self):
        self.check_profile('sim', local=True)

    def test_sim_launch(self):
        self.check_profile('sim')


if __name__ == '__main__':
    unittest.main()
