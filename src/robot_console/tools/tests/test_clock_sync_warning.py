import json
from robot_console.core.clock_sync_status import clock_sync_status


def test_readiness_requires_live_ntp_status(tmp_path):
    p=tmp_path/'status.json'
    assert not clock_sync_status(p,mono=10,boot_id='a')[0]
    good=dict(ready=True,clock_source='ntp',boot_id='a',checked_monotonic=9)
    p.write_text(json.dumps(good))
    assert clock_sync_status(p,mono=10,boot_id='a')[0]
    for change in [dict(checked_monotonic=6),dict(boot_id='b'),dict(clock_source='gnss'),
                   dict(checked_monotonic=float('nan'))]:
        p.write_text(json.dumps(dict(good,**change)))
        assert not clock_sync_status(p,mono=10,boot_id='a')[0]
    for invalid in ['null','[]','{broken']:
        p.write_text(invalid)
        assert not clock_sync_status(p,mono=10,boot_id='a')[0]


def test_warning_wait_loss_recovery_and_simulation():
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    from robot_console.ui_qt.widgets.clock_sync_warning import ClockSyncWarning
    state=[False,'NTP未同期']
    widget=ClockSyncWarning(reader=lambda:tuple(state))
    widget.set_environment('実機')
    assert not widget.isHidden() and '未成立' in widget.text()
    state[0]=True;widget.refresh();assert widget.isHidden()
    state[0]=False;widget.refresh()
    assert not widget.isHidden() and '失われました' in widget.text()
    state[0]=True;widget.refresh();assert widget.isHidden()
    widget.set_environment('デジタルツイン')
    state[0]=False;widget.refresh();assert widget.isHidden()
    widget.close()
