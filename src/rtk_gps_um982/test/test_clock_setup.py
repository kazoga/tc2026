from pathlib import Path
import subprocess
import pytest
from rtk_gps_um982.clock_setup import prepare_host
from rtk_gps_um982.ptp_trial import capture_usable


def test_host_setup_is_explicit_and_scripts_parse(tmp_path):
    p=tmp_path/'clock'
    prepare_host(p,'nkb')
    subprocess.run(['bash','-n',str(p/'apply-host.sh')],check=True)
    script=(p/'apply-host.sh').read_text()
    assert 'pgrep' in script and 'makestep' in script and '/var/backups/' in script
    assert 'chrony.service.d' in script
    assert 'noselect' in (p/'chrony.conf').read_text()
    assert 'allow ' not in (p/'chrony.conf').read_text()
    assert 'RuntimeDirectoryMode=0750' in (p/'chrony-acl.conf').read_text()
    assert 'u:nkb:rw' in (p/'chrony-acl.conf').read_text()
    assert (p/'chrony-acl.conf').read_text().count('ExecStartPost=!/usr/bin/setfacl') == 2
    assert 'ptp4l' not in script  # Apply does not broadcast a clock before convergence.
    with pytest.raises(FileExistsError):prepare_host(p,'nkb')
    with pytest.raises(ValueError):prepare_host(tmp_path/'bad','nkb;reboot')


def test_short_or_interrupted_ptp_capture_is_not_accepted():
    result=dict(ptp_seen_on_both=True, backwards={}, implausible_epoch_or_delay={},
        stream_duration_s={'lidar':59.,'imu':59.},max_receive_gap_s={'lidar':.01,'imu':.01})
    assert capture_usable(result,60.)
    for key,value in [('stream_duration_s',{'lidar':59.,'imu':.1}),
                      ('max_receive_gap_s',{'lidar':.8,'imu':.01}),
                      ('backwards',{'imu':1}),('implausible_epoch_or_delay',{'lidar':1})]:
        assert not capture_usable(dict(result,**{key:value}),60.)
