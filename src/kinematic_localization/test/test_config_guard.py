import importlib.util
import re
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_launch_module(filename: str):
    path = PACKAGE_ROOT / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ('launch_filename', 'config_filename'),
    [
        ('kinematic_localization.launch.py', 'kinematic_localization.yaml'),
        ('mapping.launch.py', 'mapping.yaml'),
    ],
)
def test_launch_rejects_missing_parameter_file(
    tmp_path: Path, launch_filename: str, config_filename: str, monkeypatch
):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    module = load_launch_module(launch_filename)
    module.get_package_share_directory = lambda _: str(tmp_path)

    with pytest.raises(RuntimeError, match='필수 파라미터 파일'):
        module.generate_launch_description()

    config_dir = tmp_path / 'config'
    config_dir.mkdir()
    (config_dir / config_filename).write_text('placeholder', encoding='utf-8')
    assert module.generate_launch_description() is not None


def test_cpp_and_yaml_schema_versions_match():
    versions = []
    for source_name in ('localization_node.cpp', 'mapping_node.cpp'):
        source = (PACKAGE_ROOT / 'src' / source_name).read_text(encoding='utf-8')
        match = re.search(r'kExpectedConfigSchemaVersion\s*=\s*(\d+)', source)
        assert match, source_name
        versions.append(int(match.group(1)))

    for config_name in ('kinematic_localization.yaml', 'mapping.yaml'):
        config = (PACKAGE_ROOT / 'config' / config_name).read_text(encoding='utf-8')
        match = re.search(r'^\s*config_schema_version:\s*(\d+)\s*$', config, re.MULTILINE)
        assert match, config_name
        versions.append(int(match.group(1)))

    assert len(set(versions)) == 1
    assert versions[0] > 0
