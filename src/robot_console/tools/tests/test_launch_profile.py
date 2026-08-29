"""LaunchProfileStore / LaunchProfile の単体テスト。"""

from pathlib import Path

from robot_console.core.launch_profile import (
    LaunchProfile,
    LaunchProfileState,
    LaunchProfileStore,
    build_initial_states,
    build_launch_args,
    build_simulator_launch_args,
    resolve_effective_overrides,
)
from robot_console.utils import NodeLaunchStatus

REPO_PROFILE_PATH = (
    Path(__file__).resolve().parents[2] / 'config' / 'node_launch_profiles.yaml'
)


def test_load_repository_profile_file_has_no_validation_errors():
    store = LaunchProfileStore(REPO_PROFILE_PATH)
    profiles = store.load()

    assert store.validation_errors == []
    assert len(profiles) == 14
    profile_ids = [profile.profile_id for profile in profiles]
    assert len(profile_ids) == len(set(profile_ids))


def test_profiles_are_sorted_by_launch_order():
    store = LaunchProfileStore(REPO_PROFILE_PATH)
    profiles = store.load()

    orders = [profile.launch_order for profile in profiles]
    assert orders == sorted(orders)


def test_rtk_gps_um982_matches_architecture_design_example():
    store = LaunchProfileStore(REPO_PROFILE_PATH)
    store.load()

    profile = store.get('rtk_gps_um982')
    assert profile is not None
    assert profile.category == 'gps_gnss'
    assert profile.package == 'rtk_gps_um982'
    assert profile.launch_file == 'rtk_gps_um982.launch.py'
    assert profile.param_argument == 'config'
    assert profile.launch_order == 20
    assert profile.health_topics == [
        '/rtk_gps/fix',
        '/rtk_gps/heading',
        '/rtk_gps/rtk_status',
    ]


def test_robot_navigator_simulator_argument_map_translates_keys():
    store = LaunchProfileStore(REPO_PROFILE_PATH)
    store.load()

    profile = store.get('robot_navigator')
    assert profile is not None
    assert profile.simulator_package == 'robot_navigator'
    assert profile.simulator_launch_file == 'robot_simulator.launch.py'
    assert profile.simulator_argument_map == {
        'pose_enu_topic': 'pose_topic',
        'odom_topic': 'odom_topic',
    }
    # cmd_vel_topic は robot_simulator.launch.py に存在しないため転送対象に含めない
    assert 'cmd_vel_topic' not in profile.simulator_argument_map


def test_obstacle_monitor_has_no_param_argument():
    """obstacle_monitor.launch.py は param_file 引数を持たない。"""

    store = LaunchProfileStore(REPO_PROFILE_PATH)
    store.load()

    profile = store.get('obstacle_monitor')
    assert profile is not None
    assert profile.param_argument is None


def test_missing_file_produces_validation_error(tmp_path: Path):
    store = LaunchProfileStore(tmp_path / 'does_not_exist.yaml')
    profiles = store.load()

    assert profiles == []
    assert len(store.validation_errors) == 1
    assert '見つかりません' in store.validation_errors[0]


def test_missing_required_field_is_reported_and_skipped(tmp_path: Path):
    path = tmp_path / 'profiles.yaml'
    path.write_text(
        "profiles:\n"
        "  - profile_id: broken\n"
        "    category: route_stack\n"
        "    package: route_manager\n"
        "    launch_file: route_manager.launch.py\n",
        encoding='utf-8',
    )
    store = LaunchProfileStore(path)
    profiles = store.load()

    assert profiles == []
    assert len(store.validation_errors) == 1
    assert 'display_name' in store.validation_errors[0]


def test_duplicate_profile_id_is_reported(tmp_path: Path):
    path = tmp_path / 'profiles.yaml'
    path.write_text(
        "profiles:\n"
        "  - profile_id: dup\n"
        "    category: route_stack\n"
        "    display_name: Dup A\n"
        "    package: route_manager\n"
        "    launch_file: route_manager.launch.py\n"
        "  - profile_id: dup\n"
        "    category: route_stack\n"
        "    display_name: Dup B\n"
        "    package: route_manager\n"
        "    launch_file: route_manager.launch.py\n",
        encoding='utf-8',
    )
    store = LaunchProfileStore(path)
    profiles = store.load()

    assert len(profiles) == 1
    assert any('重複' in message for message in store.validation_errors)


def test_health_topics_must_be_a_list(tmp_path: Path):
    path = tmp_path / 'profiles.yaml'
    path.write_text(
        "profiles:\n"
        "  - profile_id: broken\n"
        "    category: route_stack\n"
        "    display_name: Broken\n"
        "    package: route_manager\n"
        "    launch_file: route_manager.launch.py\n"
        "    health_topics: /route_state\n",
        encoding='utf-8',
    )
    store = LaunchProfileStore(path)
    profiles = store.load()

    assert profiles == []
    assert any('health_topics' in message for message in store.validation_errors)


def test_top_level_must_have_profiles_list(tmp_path: Path):
    path = tmp_path / 'profiles.yaml'
    path.write_text("not_profiles: []\n", encoding='utf-8')
    store = LaunchProfileStore(path)
    profiles = store.load()

    assert profiles == []
    assert len(store.validation_errors) == 1


def test_build_initial_states_seeds_defaults_from_profile():
    profile = LaunchProfile(
        profile_id='ypspur_ros2',
        category='real_robot_base',
        display_name='YP-Spur Coordinator',
        package='ypspur_ros2',
        launch_file='ypspur_ros2.launch.py',
        default_param='config/default.yaml',
        user_arguments=['cmd_vel_topic', 'odom_topic'],
        default_arguments={'cmd_vel_topic': '/cmd_vel'},
    )

    states = build_initial_states([profile])

    assert set(states.keys()) == {'ypspur_ros2'}
    state = states['ypspur_ros2']
    assert isinstance(state, LaunchProfileState)
    assert state.status == NodeLaunchStatus.STOPPED
    assert state.selected_param == 'config/default.yaml'
    assert state.override_inputs == {'cmd_vel_topic': '/cmd_vel', 'odom_topic': ''}


def _navigator_profile() -> LaunchProfile:
    return LaunchProfile(
        profile_id='robot_navigator',
        category='drive_stack',
        display_name='Robot Navigator',
        package='robot_navigator',
        launch_file='robot_navigator.launch.py',
        param_argument='param_file',
        default_param='params/default.yaml',
        simulator_package='robot_navigator',
        simulator_launch_file='robot_simulator.launch.py',
        simulator_argument_map={'pose_enu_topic': 'pose_topic', 'odom_topic': 'odom_topic'},
        user_arguments=['cmd_vel_topic', 'odom_topic', 'pose_enu_topic'],
        default_arguments={'cmd_vel_topic': '/cmd_vel/autonomous', 'odom_topic': '/ypspur_ros/odom'},
    )


def test_resolve_effective_overrides_prefers_state_input_over_default():
    profile = _navigator_profile()
    state = LaunchProfileState(
        profile_id='robot_navigator',
        override_inputs={
            'cmd_vel_topic': '/cmd_vel/manual',
            'odom_topic': '',
            'pose_enu_topic': '',
        },
    )

    overrides = resolve_effective_overrides(profile, state)

    # cmd_vel_topicはstate入力を優先、odom_topicはprofile既定値へfallback、
    # pose_enu_topicはどちらも無いため含めない。
    assert overrides == {'cmd_vel_topic': '/cmd_vel/manual', 'odom_topic': '/ypspur_ros/odom'}


def test_build_launch_args_uses_alternate_launch_file_when_requested():
    profile = LaunchProfile(
        profile_id='road_blockage_detector',
        category='perception_stack',
        display_name='Road Blockage Detector',
        package='road_blockage_detector',
        launch_file='road_blockage_perception.launch.py',
        alternate_launch_file='road_blockage_perception_yolo.launch.py',
        param_argument='detector_param_file',
    )

    args = build_launch_args(
        profile, param_path='params/default.yaml', use_alternate=True
    )

    assert args == [
        'ros2',
        'launch',
        'road_blockage_detector',
        'road_blockage_perception_yolo.launch.py',
        'detector_param_file:=params/default.yaml',
    ]


def test_build_launch_args_omits_param_argument_when_profile_has_none():
    profile = LaunchProfile(
        profile_id='obstacle_monitor',
        category='obstacle_stack',
        display_name='Obstacle Monitor',
        package='obstacle_monitor',
        launch_file='obstacle_monitor.launch.py',
        param_argument=None,
    )

    args = build_launch_args(profile, param_path='params/default.yaml')

    assert args == ['ros2', 'launch', 'obstacle_monitor', 'obstacle_monitor.launch.py']


def test_build_simulator_launch_args_only_forwards_mapped_overrides():
    profile = _navigator_profile()

    args = build_simulator_launch_args(
        profile,
        {
            'cmd_vel_topic': '/cmd_vel/autonomous',
            'odom_topic': '/ypspur_ros/odom',
            'pose_enu_topic': '/localization/pose_enu',
        },
    )

    assert args[:4] == ['ros2', 'launch', 'robot_navigator', 'robot_simulator.launch.py']
    assert not any(arg.startswith('cmd_vel_topic:=') for arg in args)
    assert 'odom_topic:=/ypspur_ros/odom' in args
    assert 'pose_topic:=/localization/pose_enu' in args
