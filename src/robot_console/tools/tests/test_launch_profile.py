"""LaunchProfileStore / LaunchProfile の単体テスト。"""

from pathlib import Path

from robot_console.core.launch_profile import (
    LaunchProfile,
    LaunchProfileState,
    LaunchProfileStore,
    build_initial_states,
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
