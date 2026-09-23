import math

import numpy as np
import pytest

from route_survey.free_space_core import (
    DROP, GROUND, MATERIAL, OBSTACLE, GroundThresholds, SpaceConfig, build_space,
    choose_path, space_from_labels,
)


def corridor(width=2.4, wall=None, unseen=False):
    config = SpaceConfig()
    xx, yy = np.meshgrid(np.arange(config.xmin+.075, config.xmax, .15),
                         np.arange(config.ymin+.075, config.ymax, .15))
    labels = np.where(abs(yy) < width/2, GROUND, MATERIAL).astype(np.uint8)
    if wall is not None:
        labels[(xx > wall) & (xx < wall+.3)] = OBSTACLE
    if unseen:
        labels[xx > 1.] = 0
    return space_from_labels(labels, config)


def test_straight_corridor_preserves_progress_and_footprint():
    result = choose_path(corridor())
    assert result['status'] == 'candidate'
    assert abs(result['curvature']) < .06
    assert result['clearance_m'] >= .425
    assert result['path'][-1][0] > 2.5


@pytest.mark.parametrize('bias', [-1., 1.])
def test_uncertain_lateral_route_does_not_force_robot_off_ground(bias):
    result = choose_path(corridor(), lateral_target=bias, position_sigma=1.)
    assert result['status'] == 'candidate'
    path = np.array(result['path'])
    assert np.max(abs(path[:, 1]))+.4 <= 1.2


@pytest.mark.parametrize('space', [corridor(.6), corridor(wall=.8), corridor(unseen=True)])
def test_narrow_blocked_or_unobserved_corridor_stops(space):
    result = choose_path(space)
    assert result['status'] == 'stop' and result['speed'] == 0.


@pytest.mark.parametrize('age', [-.1, .51, math.inf, math.nan])
def test_invalid_or_stale_data_stops(age):
    assert choose_path(corridor(), observation_age=age)['status'] == 'stop'


def test_large_route_direction_error_requires_reorientation():
    assert choose_path(corridor(), route_heading=math.pi)['status'] == 'stop'


def test_old_ground_survives_a_current_object_but_path_cannot_enter_it():
    xx, yy = np.meshgrid(np.arange(-1.2, 5.1, .025), np.arange(-3.6, 3.6, .025))
    cloud = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -.15), np.full(xx.size, 15.)])
    # Preserve pavement evidence beneath a cone and overlay its current occupied cell.
    empty = build_space(cloud, np.empty((0, 2)))
    occupied = build_space(cloud, np.array([[1.5, 0.]]))
    ix, iy, _ = occupied.indices(np.array([[1.5, 0.]]))
    assert empty.labels[iy[0], ix[0]] == GROUND
    assert occupied.labels[iy[0], ix[0]] == OBSTACLE
    result = choose_path(occupied)
    if result['status'] == 'candidate':
        assert np.linalg.norm(np.asarray(result['path'])-[1.5, 0.], axis=1).min() > .4


def test_absent_material_evidence_is_not_free_space():
    assert build_space(np.empty((0, 4)), np.empty((0, 2))).reason != 'ok'
    with pytest.raises(ValueError):
        build_space(np.empty((0, 3)), np.empty((0, 2)))


def test_threshold_relaxation_cannot_remove_drop_current_obstacle_or_fill_unseen_cell():
    xx, yy = np.meshgrid(np.arange(-1.175, 5.1, .05), np.arange(-3.575, 3.6, .05))
    cloud = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -.15), np.full(xx.size, 15.)])
    # Three independent locations, away from the reference plane seed.
    probes = np.array([[1.575, 1.575], [1.575, 1.125], [1.575, -.975]])
    for point, action in zip(probes, ['drop', 'obstacle', 'unseen']):
        mask = (abs(cloud[:, 0]-point[0]) < .074) & (abs(cloud[:, 1]-point[1]) < .074)
        if action == 'drop':
            cloud[mask, 2] -= .15
        elif action == 'unseen':
            cloud = cloud[~mask]
    permissive = GroundThresholds(material_ratio=5., material_points=6,
                                   material_fraction=.2, rough_height=.1, support_points=2)
    space = build_space(cloud, probes[1:2], thresholds=permissive)
    ix, iy, _ = space.indices(probes)
    assert space.labels[iy, ix].tolist() == [DROP, OBSTACLE, 0]


def test_intensity_relaxation_exposes_flat_grass_counterexample():
    xx, yy = np.meshgrid(np.arange(-1.175, 5.1, .05), np.arange(-3.575, 3.6, .05))
    # Known synthetic grass lies outside y=1.2, with identical geometric shape.
    cloud = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -.15),
                             np.where(yy.ravel() > 1.2, 45., 15.)])
    baseline = build_space(cloud, np.empty((0, 2)))
    relaxed = build_space(cloud, np.empty((0, 2)), thresholds=GroundThresholds(material_ratio=3.5))
    ix, iy, _ = baseline.indices(np.array([[1.575, 1.575]]))
    assert baseline.labels[iy[0], ix[0]] == MATERIAL
    assert relaxed.labels[iy[0], ix[0]] == GROUND


@pytest.mark.parametrize('kwargs', [dict(material_ratio=float('nan')), dict(support_points=0),
                                   dict(support_points=2.5), dict(rough_height=.05),
                                   dict(support_height=.09), dict(material_fraction=1.1)])
def test_invalid_thresholds_are_rejected(kwargs):
    with pytest.raises(ValueError):
        GroundThresholds(**kwargs)
