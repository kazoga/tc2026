import numpy as np
import pytest
from route_survey.surface_core import surface_width,SurfaceWindow


def scene(kind='road'):
    x,y=np.meshgrid(np.arange(-.595,3.,.04),np.arange(-2.99,3.,.04));z=-.15+.015*x
    intensity=np.full(x.shape,15.)
    if kind=='grass':intensity[:]=80.
    side=y< -1.
    if kind in ['boundary','grass']:z=z+np.where(side,.02,0)
    if kind=='boundary':intensity[side]=80.
    if kind=='curb':z=z+np.where(side,.12,0)
    if kind=='drop':z=z-np.where(side,.12,0)
    return np.column_stack([x.ravel(),y.ravel(),z.ravel(),intensity.ravel()])


def test_asphalt_grass_boundary_does_not_expand_into_grass():
    result=surface_width(scene('boundary'))
    assert result['left']>2.5
    assert .2<result['right']<=.65+1e-6
    assert result['right_reason']=='material_change'


def test_grass_under_robot_is_not_a_safe_seed():
    result=surface_width(scene('grass'))
    assert result['left']==result['right']==0
    assert result['left_reason']=='no_surface_reference'


@pytest.mark.parametrize('kind',['curb','drop'])
def test_edges_remain_blocked(kind):
    result=surface_width(scene(kind))
    assert result['right']<=.65+1e-6
    assert result['left']>2.5


def test_unobserved_patch_is_not_filled():
    p=scene();p=p[~((p[:,0]<-.2)&(p[:,1]>.4)&(p[:,1]<.6))]
    result=surface_width(p)
    assert result['left']<=.05+1e-6 and result['left_reason']=='unknown'


def test_short_window_expires_and_resets_on_jump():
    w=SurfaceWindow(seconds=1.)
    p=scene();p[:,:3]+=[10,20,0]
    local=w.update(1.,p,[10,20,0],0)
    assert surface_width(local)['left']>2.5
    w.update(1.5,np.empty((0,4)),[10,20,0],0)
    assert len(w.update(2.1,np.empty((0,4)),[10,20,0],0))==0
    w.update(2.2,p,[10,20,0],0)
    assert len(w.update(2.3,np.empty((0,4)),[30,20,0],0))==0


def test_result_is_saveable_and_central_hazard_disables_both_sides():
    import json
    p=scene();p[(p[:,1]>0)&(p[:,1]<.2),2]+=.15
    result=surface_width(p,details=True)
    assert result['left']==result['right']==0
    json.dumps(result,allow_nan=False)
    json.dumps(surface_width(scene(),details=True),allow_nan=False)
