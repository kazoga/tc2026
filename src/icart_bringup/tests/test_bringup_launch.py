"""launch記述を展開し、実機と模擬の配信元を分離できることを確認する."""
import importlib.util
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from icart_bringup.session_core import validate_domain


def test_domain_mismatch_and_shared_domain_are_rejected() -> None:
    validate_domain({}, 'simulation', 86)
    validate_domain({}, 'real', 0)
    with pytest.raises(ValueError, match='ROS_DOMAIN_ID'):
        validate_domain({}, 'simulation', 0)
    with pytest.raises(ValueError, match='異なる'):
        validate_domain({'simulation_domain_id': 86, 'real_domain_id': 86}, 'real', 86)


def test_real_launch_has_no_simulator_and_sim_has_one_fusion(monkeypatch, tmp_path: Path) -> None:
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        'icart_launch', Path(__file__).parents[1]/'launch/bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []

    class CapturingNode(Node):
        def __init__(self, **kwargs) -> None:
            captured.append(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(module, 'Node', CapturingNode)
    data = dict(projection_params='projection.yaml', route_config='route.yaml',
                csv_base_dir=str(tmp_path), trial_directory=str(tmp_path), goal_label='10')
    monkeypatch.setattr(module, 'load_session', lambda *_: data)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    for mode, domain in [('real', '0'), ('simulation', '86')]:
        captured.clear()
        monkeypatch.setenv('ROS_DOMAIN_ID', domain)
        context = LaunchContext()
        context.launch_configurations.update(environment=mode, session='session.yaml',
                                              start_ui='true', fusion_log='', initial_drive_mode='autonomous')
        actions = module.setup(context)
        packages = [a.node_package
                    for a in actions if isinstance(a, Node)]
        assert packages.count('gnss_lio_fusion') == 1
        assert packages.count('drive_mode_manager') == 1
        assert packages.count('fast_lio') == 1
        assert ('obstacle_route_sim' in packages) == (mode == 'simulation')
        assert ('ros_gz_bridge' in packages) == (mode == 'simulation')
        navigator = next(item for item in captured if item['package'] == 'robot_navigator')
        params = {key: value for entry in navigator['parameters'] if isinstance(entry, dict)
                  for key, value in entry.items()}
        assert params['pose_timeout_sec'] == 1.
        assert params['odom_timeout_sec'] == 1.
        ui = next(item for item in captured if item['package'] == 'robot_console')
        assert ui['arguments'] == ['--business-environment',
                                   'デジタルツイン' if mode == 'simulation' else '実機（融合）']
