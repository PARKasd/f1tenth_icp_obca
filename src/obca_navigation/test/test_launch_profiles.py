"""Evaluate launch wiring/parameter precedence without claiming a running ROS graph."""
import ast
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def load(filename):
    modules = {}
    exports = {
        'ament_index_python.packages': {
            'get_package_share_directory': lambda name: str(PACKAGE.parent / name)},
        'launch': {'LaunchDescription': lambda actions: actions},
        'launch.actions': {name: Action for name in
                           ('DeclareLaunchArgument', 'OpaqueFunction', 'IncludeLaunchDescription')},
        'launch.substitutions': {'LaunchConfiguration': Configuration},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': Action},
        'launch_ros.actions': {'Node': Action},
    }
    for name, values in exports.items():
        modules[name] = types.ModuleType(name)
        modules[name].__dict__.update(values)
    spec = importlib.util.spec_from_file_location(filename, PACKAGE / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def parameters(node):
    result = {}
    name = node.kwargs['name']
    for layer in node.kwargs['parameters']:
        if isinstance(layer, dict):
            result.update(layer)
        else:
            contents = yaml.safe_load(Path(layer).read_text(encoding='utf-8'))
            for key in ('/**', name):
                result.update(contents.get(key, {}).get('ros__parameters', {}))
    return result


class LaunchProfiles(unittest.TestCase):
    def setUp(self):
        self.common = load('navigation.launch.py')
        self.context = {a.args[0]: a.kwargs['default_value']
                        for a in self.common.generate_launch_description() if a.args}

    def nodes(self, profile='', **overrides):
        context = dict(self.context)
        context['profile_file'] = str(PACKAGE / 'config' / profile) if profile else ''
        context.update(overrides)
        return {node.kwargs['name']: parameters(node) for node in self.common._nodes(context)}

    def test_python_and_yaml_syntax(self):
        for path in (PACKAGE / 'launch').glob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        for path in (PACKAGE / 'config').glob('*.yaml'):
            self.assertIsInstance(yaml.safe_load(path.read_text(encoding='utf-8')), dict)

    def test_wrapper_profiles(self):
        for mode in ('real', 'sim'):
            include, = load(f'navigation_{mode}.launch.py').generate_launch_description()
            self.assertEqual(dict(include.kwargs['launch_arguments'])['profile_file'],
                             str(PACKAGE / 'config' / f'{mode}.yaml'))
            self.assertEqual(include.args[0].args[0], str(PACKAGE / 'launch' / 'navigation.launch.py'))

    def test_real_profile(self):
        nodes = self.nodes('real.yaml')
        for node in nodes.values():
            self.assertIs(node['use_sim_time'], False)
            self.assertEqual(node['base_frame'], 'base_link')
        for name in ('obca_planner', 'obca_tracker'):
            self.assertEqual(nodes[name]['drive_topic'], '/drive_autonomous')
        self.assertEqual(nodes['kinematic_localization']['odom_topic'], '/odom')
        self.assertIs(nodes['kinematic_localization']['publish_map_odom_tf'], True)

    def test_sim_profile(self):
        nodes = self.nodes('sim.yaml')
        for node in nodes.values():
            self.assertIs(node['use_sim_time'], True)
            self.assertEqual(node['base_frame'], 'ego_racecar/base_link')
        for name in ('obca_planner', 'obca_tracker'):
            self.assertEqual(nodes[name]['drive_topic'], '/drive')
        self.assertEqual(nodes['kinematic_localization']['odom_topic'], '/ego_racecar/odom')
        self.assertIs(nodes['kinematic_localization']['publish_map_odom_tf'], False)

    def test_common_preserves_preview_output(self):
        nodes = self.nodes()
        self.assertEqual(nodes['obca_tracker']['drive_topic'], '/obca/drive')
        self.assertIs(nodes['kinematic_localization']['require_initial_pose'], True)
        self.assertIs(nodes['kinematic_localization']['slam_mode'], True)

    def test_cli_overrides_profile_for_all_consumers(self):
        nodes = self.nodes('sim.yaml', use_sim_time='false', drive_topic='/test/drive',
                           wheel_odom_topic='/test/odom', base_frame='test_base',
                           scan_topic='/test/scan', publish_map_odom_tf='true')
        for node in nodes.values():
            self.assertIs(node['use_sim_time'], False)
            self.assertEqual(node['base_frame'], 'test_base')
        self.assertEqual(nodes['obca_planner']['scan_topic'], '/test/scan')
        self.assertEqual(nodes['obca_tracker']['drive_topic'], '/test/drive')
        self.assertEqual(nodes['obca_planner']['drive_topic'], '/test/drive')
        self.assertEqual(nodes['kinematic_localization']['lidar_topic'], '/test/scan')
        self.assertEqual(nodes['kinematic_localization']['odom_topic'], '/test/odom')
        self.assertIs(nodes['kinematic_localization']['publish_map_odom_tf'], True)

    def test_missing_profile_and_invalid_boolean_fail(self):
        with self.assertRaises(RuntimeError):
            self.nodes('missing.yaml')
        with self.assertRaises(ValueError):
            self.nodes('sim.yaml', use_sim_time='tru')
        with self.assertRaises(ValueError):
            self.nodes('real.yaml', publish_map_odom_tf='yes')


if __name__ == '__main__':
    unittest.main()
