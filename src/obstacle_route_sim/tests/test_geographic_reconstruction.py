"""座標の往復、誤検出除外、候補の領域制限を確認する."""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from geographic_reconstruction_core import (
    filter_tree_candidates, pixel_llh, route_from_pixels, vehicle_candidates, world_pixels)


def test_mercator_round_trip_across_course() -> None:
    lat=np.array([36.0818,36.08375,36.0857]);lon=np.array([140.075,140.0795,140.084])
    x,y=world_pixels(lat,lon,19);actual_lat,actual_lon=pixel_llh(x,y,19)
    np.testing.assert_allclose(actual_lat,lat,atol=1e-10,rtol=0)
    np.testing.assert_allclose(actual_lon,lon,atol=1e-10,rtol=0)
    assert y[0]>y[-1]


def test_invalid_registration_is_rejected() -> None:
    with pytest.raises(ValueError):
        route_from_pixels({'official_route_pixels':[[0,0],[1,1]]},
                          {'affine':[[float('nan'),0,0],[0,1,0]]},
                          {'x0':0,'y0':0,'z':18})


def test_parking_mask_prevents_vehicle_detections_outside_parking() -> None:
    image=np.zeros((100,100,3),np.uint8)
    image[20:38,20:60]=220
    mask=np.full((100,100),255,np.uint8)
    assert len(vehicle_candidates(image,mask,.1))==1
    mask[:]=0
    assert vehicle_candidates(image,mask,.1)==[]
    with pytest.raises(ValueError):vehicle_candidates(image,mask,0)


def test_exclusion_removes_green_roof_tree_candidate() -> None:
    image=np.zeros((100,100,3),np.uint8);image[:]=[45,130,40]
    candidate=dict(x=0.,y=0.,r=2.,h=6.,confidence=.8)
    exclusion=np.zeros((100,100),np.uint8)
    assert len(filter_tree_candidates(image,[candidate],exclusion,20,20))==1
    exclusion[40:60,40:60]=1
    assert filter_tree_candidates(image,[candidate],exclusion,20,20)==[]
