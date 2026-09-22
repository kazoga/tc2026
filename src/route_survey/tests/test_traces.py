from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parents[1]))
from route_survey.trace_core import Traces


def test_status_time_matching_and_missing():
    t=Traces()
    for stamp in [1.,2.,3.]:t.append('gnss',dict(stamp=stamp,x=0,y=0))
    t.append('statuses',dict(stamp=1.1,state=4))
    t.append('statuses',dict(stamp=2.1,state=3))
    rows=t.export()['gnss']
    assert [r['quality']['state'] if r['quality'] else None for r in rows]==[4,3,None]


def test_limit_and_independent_tracks():
    t=Traces(limit=2)
    for i in range(3):t.append('fused',dict(stamp=float(i),x=i,y=0))
    t.append('gnss',dict(stamp=1.,x=None,y=None))
    assert len(t.export()['fused'])==2 and t.dropped==1
    assert t.export()['gnss'][0]['x'] is None
