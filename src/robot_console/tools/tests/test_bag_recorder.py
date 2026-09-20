import time
from types import SimpleNamespace
import pytest
from PyQt5 import QtWidgets
from robot_console.core.bag_recorder import BagRecorder, BagState
from robot_console.ui_qt.widgets.bag_card import BagCard


def test_card_disables_duplicate_start_and_preserves_destination():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    card = BagCard()
    card.update_state(BagState(directory='/tmp/bags'))
    card.directory.setText('/tmp/my_bags')
    calls = []
    card.start_requested.connect(calls.append)
    card.start_button.click()
    assert calls == ['/tmp/my_bags']
    card.update_state(BagState(state='RECORDING', directory='/tmp/bags'))
    assert card.directory.text() == '/tmp/my_bags'
    assert not card.start_button.isEnabled() and card.stop_button.isEnabled()
    card.update_state(BagState(state='STOPPING'))
    assert not card.start_button.isEnabled() and not card.stop_button.isEnabled()
    card.update_state(BagState(state='SAVED'))
    assert card.start_button.isEnabled()


def test_low_disk_refuses_recording_without_process(tmp_path, monkeypatch):
    monkeypatch.setattr('robot_console.core.bag_recorder.shutil.disk_usage',
                        lambda _: SimpleNamespace(free=100))
    recorder = BagRecorder(tmp_path)
    assert not recorder.start()
    assert recorder.snapshot().state == 'ERROR'
    assert '1 GiB' in recorder.snapshot().message
    assert recorder._file_lock is None


def test_missing_ros2_is_reported_and_lock_released(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise FileNotFoundError('ros2 is unavailable')
    monkeypatch.setattr('robot_console.core.bag_recorder.subprocess.Popen', fail)
    recorder = BagRecorder(tmp_path)
    assert not recorder.start()
    assert recorder.snapshot().state == 'ERROR'
    assert recorder._file_lock is None


def test_real_bag_records_messages_and_finalizes_on_close(tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from std_msgs.msg import String
    import sqlite3
    monkeypatch.setenv('ROS_DOMAIN_ID', '193')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    ctx = Context()
    rclpy.init(context=ctx, domain_id=193)
    node = rclpy.create_node('bag_ui_test_source', context=ctx)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    pub = node.create_publisher(String, '/bag_ui_test', 10)
    recorder = BagRecorder(tmp_path)
    second = BagRecorder(tmp_path)
    try:
        assert recorder.start()
        assert not recorder.start()  # 同一UIの重複操作
        assert not second.start()    # 同じ保存先を使う別UI
        assert '別のUI' in second.snapshot().message
        deadline = time.monotonic()+12.
        received_subscription = None
        while time.monotonic() < deadline:
            pub.publish(String(data='bag UI integration test'))
            executor.spin_once(timeout_sec=.05)
            if pub.get_subscription_count():
                received_subscription = received_subscription or time.monotonic()
                if time.monotonic()-received_subscription > 1.:
                    break
        assert received_subscription is not None
        recorder.close()
        state = recorder.snapshot()
        assert state.state == 'SAVED', state
        from pathlib import Path
        output = Path(state.output)
        assert (output/'metadata.yaml').is_file()
        count = 0
        for db in output.glob('*.db3'):
            with sqlite3.connect(db) as conn:
                count += conn.execute('SELECT COUNT(*) FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=?',
                                      ('/bag_ui_test',)).fetchone()[0]
        assert count > 0
    finally:
        recorder.close()
        executor.shutdown()
        node.destroy_node()
        ctx.shutdown()
