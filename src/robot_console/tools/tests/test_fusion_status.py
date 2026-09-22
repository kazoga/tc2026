"""融合表示の変換、異常入力、UIとの接続を確認する."""
import json
from types import SimpleNamespace

from robot_console.core.console_core import ConsoleCore
from robot_console.core.freshness import FreshnessLevel
from robot_console.core.business_mode import get_preset


def test_fusion_view_and_invalid_data() -> None:
    core = ConsoleCore()
    core.update_fusion_status(SimpleNamespace(data=json.dumps(dict(
        mode='LIO_PRIORITY', yaw=1., heading_sigma_deg=2., baseline={'reference_m': .515}))))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'LIO_PRIORITY'
    assert 57 < state.yaw_deg < 58
    assert state.freshness == FreshnessLevel.OK
    for data in ['null', '[]', '{', '{"yaw":"nan"}']:
        core.update_fusion_status(SimpleNamespace(data=data))
    assert core.build_snapshot().fusion_state.baseline_m == .515


def test_fused_presets_only_select_one_stack() -> None:
    for mode, environment in [('実機（融合）', 'real'), ('デジタルツイン', 'simulation')]:
        preset = get_preset(mode, '自律走行')
        assert len(preset) == 1
        assert preset[0].profile_id == ('icart_recorded_route' if environment == 'real' else 'icart_fused_stack')
        assert preset[0].overrides == ({} if environment == 'real' else {'environment': environment})


def test_initial_fix_wait_has_no_invented_heading() -> None:
    core = ConsoleCore()
    core.update_fusion_status(SimpleNamespace(data=json.dumps(dict(
        mode='WAIT_INITIAL_FIX', yaw=None, heading_sigma_deg=None, baseline={'reference_m': .5}))))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'WAIT_INITIAL_FIX'
    assert state.yaw_deg is None and state.heading_sigma_deg is None
    assert state.freshness == FreshnessLevel.OK


def test_gravity_wait_is_visible_without_a_heading() -> None:
    core = ConsoleCore()
    core.update_fusion_status(SimpleNamespace(data=json.dumps(dict(
        mode='WAIT_GRAVITY_ALIGNMENT', baseline={'reference_m': .15}))))
    state = core.build_snapshot().fusion_state
    assert state.mode == 'WAIT_GRAVITY_ALIGNMENT'
    assert state.yaw_deg is None and state.heading_sigma_deg is None
    assert state.freshness == FreshnessLevel.OK
