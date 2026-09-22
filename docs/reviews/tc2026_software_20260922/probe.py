"""No ROS nodes, publishers, or motor connections: reproduce current boundary behavior."""
from pathlib import Path
import sys, json, ast
from types import SimpleNamespace
import numpy as np
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/route_follower'))
from route_follower.follower_core import FollowerCore, Pose, Waypoint, Route
logger = SimpleNamespace(info=lambda *args: None)
results = {}
for stop in (True, False):
    c = FollowerCore(logger)
    c.update_route(Route(1, [Waypoint('target', Pose(1,0), line_stop=stop)]))
    c.update_pose(Pose(0,0)); c.tick()
    c.update_pose(Pose(.45,0)); out = c.tick()
    results['stop' if stop else 'finish'] = dict(distance_m=.55, status=c.status.name,
        target_returned=out.target_pose is not None, navigator_default_pos_tol_m=.5)
assert results['stop']['status'] == 'WAITING_STOP'
assert results['finish']['status'] == 'FINISHED'
c = FollowerCore(logger)
c._start_avoidance_sequence(Waypoint('narrow',Pose(2,0),left_open=.1),Pose(0,0),.1,0)
results['avoidance'] = dict(allowed_m=.1, generated_m=abs(c.last_applied_offset_m))
assert results['avoidance']['generated_m'] == .35
# Extract this pure method from source to avoid importing/initializing ROS or camera libraries.
p = ROOT/'src/obstacle_monitor/obstacle_monitor/obstacle_monitor_node.py'
cls = next(n for n in ast.parse(p.read_text()).body if isinstance(n, ast.ClassDef) and n.name=='ObstacleMonitorNode')
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='_is_front_blocked')
method.decorator_list=[]
namespace={'np':np, 'Tuple':tuple}
exec(compile(ast.Module(body=[method],type_ignores=[]), str(p), 'exec'), namespace)
blocked, distance = namespace['_is_front_blocked'](None,np.empty((0,2)),.6,.75)
results['empty_obstacle_points'] = dict(blocked=blocked, clearance='infinity' if np.isinf(distance) else distance)
assert blocked is False and np.isinf(distance)
results['ideal_braking_at_1mps'] = dict(navigator_m=1/(2*1.0), driver_m=1/(2*.3),
    note='Ideal constant deceleration only; excludes latency, margin, and actual measured dynamics.')
print(json.dumps(results,ensure_ascii=False,indent=2))
