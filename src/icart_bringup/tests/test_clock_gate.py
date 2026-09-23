import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parents[1]))
from icart_bringup.clock_gate import ready


def test_gate_requires_fresh_same_boot_positive_status(tmp_path):
    p=tmp_path/'status.json'
    assert not ready(p,10.,'boot')
    for status in [dict(clock_source='ntp',ready=False,checked_monotonic=9.,boot_id='boot'),
                   dict(clock_source='ntp',ready=True,checked_monotonic=1.,boot_id='boot'),
                   dict(clock_source='ntp',ready=True,checked_monotonic=11.,boot_id='boot'),
                   dict(clock_source='ntp',ready=True,checked_monotonic=9.,boot_id='previous'),
                   dict(clock_source='ntp',ready=True,checked_monotonic=float('nan'),boot_id='boot')]:
        p.write_text(json.dumps(status));assert not ready(p,10.,'boot')
    p.write_text(json.dumps(dict(clock_source='ntp',ready=True,checked_monotonic=9.,boot_id='boot')))
    assert ready(p,10.,'boot')
    state=json.loads(p.read_text());state['clock_source']='gnss'
    p.write_text(json.dumps(state));assert not ready(p,10.,'boot')


def test_real_stack_defers_motion_nodes_until_clock_gate(monkeypatch,tmp_path):
    import importlib.util
    from types import SimpleNamespace
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch.actions import ExecuteProcess
    filename=Path(__file__).parents[1]/'launch/bringup.launch.py'
    spec=importlib.util.spec_from_file_location('clock_gated_bringup',filename)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    data=dict(projection_params='projection.yaml',route_config='route.yaml',
        csv_base_dir=str(tmp_path),goal_label='10',hardware_config=str(tmp_path/'hardware.yaml'),
        hardware_launch='hardware.launch.py')
    monkeypatch.setattr(module,'load_session',lambda *_:data)
    monkeypatch.setattr(module,'validate_domain',lambda *_:None)
    monkeypatch.setattr(module,'validate_runtime',lambda *_:None)
    monkeypatch.setattr(module,'read_yaml',lambda *_:dict(joy=dict(enable_button=4,ps_button=10)))
    monkeypatch.setattr(module,'get_package_share_directory',lambda _:str(tmp_path))
    callbacks=[];handler=module.OnProcessExit
    def capture(**kwargs):
        if callable(kwargs['on_exit']):callbacks.append(kwargs['on_exit'])
        return handler(**kwargs)
    monkeypatch.setattr(module,'OnProcessExit',capture)
    context=LaunchContext();context.launch_configurations.update(environment='real',session='session.yaml',
        initial_drive_mode='autonomous',start_ui='false',fusion_log='')
    initial=module.setup(context)
    assert not any(isinstance(action,Node) for action in initial)
    assert sum(isinstance(action,ExecuteProcess) for action in initial)==1
    assert len(callbacks)==1
    failed=callbacks[0](SimpleNamespace(returncode=1),context)
    assert not any(isinstance(action,Node) for action in failed)
    started=callbacks[0](SimpleNamespace(returncode=0),context)
    assert any(isinstance(action,Node) for action in started)
