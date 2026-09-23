"""Offline local-space prototype. This module never publishes a motion command.

Ground evidence and current obstacles are separate. A path must have observed
support beneath its full circular footprint; no-return is not free ground.
Intensity is only a site-specific material veto, not a semantic classifier.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from .surface_core import surface_width

UNKNOWN, GROUND, MATERIAL, OBSTACLE, DROP = range(5)


@dataclass(frozen=True)
class GroundThresholds:
    """Experimental perception parameters; defaults reproduce the first sample.

    These do not change obstacle, drop, footprint or unknown-space constraints.
    Increasing a threshold is an experiment, not evidence of safe pavement.
    """
    max_seed_intensity: float = 30.
    material_ratio: float = 2.5
    support_height: float = .055
    support_points: int = 3
    material_points: int = 3
    material_fraction: float = 0.
    rough_height: float = .065
    rough_points: int = 3

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Thresholds must be finite')
        if (not 0 < self.max_seed_intensity <= 255 or self.material_ratio <= 0
                or not 0 < self.support_height < .08
                or not self.support_height < self.rough_height < .12
                or not 0 <= self.material_fraction <= 1):
            raise ValueError('Invalid ground thresholds')
        if any(type(v) is not int or v < 1 for v in
               (self.support_points, self.material_points, self.rough_points)):
            raise ValueError('Point thresholds must be positive integers')


@dataclass(frozen=True)
class SpaceConfig:
    resolution: float = .15
    xmin: float = -1.2
    xmax: float = 5.1
    ymin: float = -3.6
    ymax: float = 3.6
    radius: float = .40  # Prototype footprint incl. margin; verify for the vehicle.
    horizon: float = 3.0
    min_progress: float = 1.0
    sample_step: float = .05
    max_curvature: float = 1.0
    max_speed: float = .35
    max_yaw_rate: float = .5
    deceleration: float = .5
    reaction_time: float = .3
    max_age: float = .5

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Configuration must be finite')
        if (self.resolution <= 0 or self.radius <= 0 or self.sample_step <= 0
                or self.xmin >= 0 or self.xmax <= 0 or self.ymin >= 0 or self.ymax <= 0
                or not 0 < self.min_progress <= self.horizon or self.max_curvature <= 0
                or min(self.max_speed, self.max_yaw_rate, self.deceleration, self.max_age) <= 0
                or self.reaction_time < 0):
            raise ValueError('Invalid local-space configuration')


@dataclass
class LocalSpace:
    labels: np.ndarray
    clearance: np.ndarray
    config: SpaceConfig
    reason: str = 'ok'
    plane: list | None = None

    def indices(self, xy):
        xy = np.asarray(xy, dtype=float)
        ix = np.floor((xy[:, 0]-self.config.xmin)/self.config.resolution).astype(int)
        iy = np.floor((xy[:, 1]-self.config.ymin)/self.config.resolution).astype(int)
        inside = ((ix >= 0) & (ix < self.labels.shape[1]) &
                  (iy >= 0) & (iy < self.labels.shape[0]))
        return ix, iy, inside


def space_from_labels(labels, config=SpaceConfig(), reason='ok', plane=None):
    """Treat the outside of the grid as unknown, including all-free test grids."""
    labels = np.asarray(labels, dtype=np.uint8)
    # Bound both the obstacle cell extent and the query's position inside its cell.
    distance = distance_transform_edt(np.pad(labels == GROUND, 1))[1:-1, 1:-1]
    clearance = np.maximum(0., distance*config.resolution-config.resolution*math.sqrt(2))
    return LocalSpace(labels, clearance, config, reason, plane)


def build_space(ground_history, current_obstacles, config=SpaceConfig(),
                thresholds=GroundThresholds()):
    """XYZI ground history in level base axes, and current XY obstacle returns.

    Object returns do not erase previously seen ground. The separate obstacle
    layer still makes those cells impassable in this local planning snapshot.
    No interpolation is used to fill unknown cells.
    """
    shape = (math.ceil((config.ymax-config.ymin)/config.resolution),
             math.ceil((config.xmax-config.xmin)/config.resolution))
    labels = np.zeros(shape, dtype=np.uint8)
    empty = space_from_labels(labels, config)
    points = np.asarray(ground_history, dtype=float)
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError('Ground history must be an N by 4 XYZI array')
    points = points[np.isfinite(points).all(axis=1)]
    model = surface_width(points, max_seed_intensity=thresholds.max_seed_intensity,
                          material_ratio=thresholds.material_ratio)
    if 'plane' not in model or not model.get('material_checked'):
        empty.reason = model.get('left_reason', 'no_surface_reference')
        return empty
    plane = np.asarray(model['plane'])
    residual = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
    ix, iy, inside = empty.indices(points[:, :2])
    index = iy[inside]*shape[1]+ix[inside]
    z = residual[inside]
    intensity = points[inside, 3]
    limit = model['intensity_reject_above']

    def counts(mask):
        return np.bincount(index[mask], minlength=labels.size).reshape(shape)

    support = counts((abs(z) <= thresholds.support_height) & (intensity <= limit))
    material = counts((abs(z) <= .12) & (intensity > limit))
    near_surface = counts(abs(z) <= .12)
    drop = counts((z < -.08) & (z > -.6))
    rough = counts((z > thresholds.rough_height) & (z <= .12))
    labels[support >= thresholds.support_points] = GROUND
    material_veto = ((material >= thresholds.material_points)
                     & (material >= thresholds.material_fraction*near_surface))
    labels[material_veto | (rough >= thresholds.rough_points)] = MATERIAL
    labels[drop >= 3] = DROP
    obstacles = np.asarray(current_obstacles, dtype=float)
    if obstacles.ndim != 2 or obstacles.shape[1] != 2:
        raise ValueError('Obstacles must be an N by 2 XY array')
    obstacles = obstacles[np.isfinite(obstacles).all(axis=1)]
    ox, oy, valid = empty.indices(obstacles)
    labels[oy[valid], ox[valid]] = OBSTACLE
    return space_from_labels(labels, config, plane=plane.tolist())


def arc(curvature, length, step=.05):
    distance = np.linspace(0., length, max(2, math.ceil(length/step)+1))
    if abs(curvature) < 1e-10:
        return np.column_stack([distance, np.zeros_like(distance)])
    return np.column_stack([np.sin(curvature*distance)/curvature,
                            (1-np.cos(curvature*distance))/curvature])


def choose_path(space, route_heading=0., lateral_target=0., position_sigma=.8,
                previous_curvature=0., observation_age=0.):
    """Rank footprint-checked arcs, using route direction as a soft objective.

    Output is a diagnostic suggestion, not a validated controller. Humans are
    treated as current obstacles; their future motion is not predicted.
    """
    config = space.config
    stop = dict(status='stop', reason=space.reason, curvature=0., speed=0.,
                yaw_rate=0., path=[], clearance_m=0., valid_candidates=0)
    inputs = (route_heading, lateral_target, position_sigma, previous_curvature, observation_age)
    if not all(math.isfinite(v) for v in inputs) or position_sigma < 0:
        return dict(stop, reason='invalid_input')
    if not 0 <= observation_age <= config.max_age:
        return dict(stop, reason='stale_observation')
    route_heading = math.atan2(math.sin(route_heading), math.cos(route_heading))
    if abs(route_heading) > math.radians(65):
        return dict(stop, reason='route_direction_outside_forward_sector')
    route = np.array([math.cos(route_heading), math.sin(route_heading)])
    candidates = []
    for curvature in np.linspace(-config.max_curvature, config.max_curvature, 41):
        path = arc(curvature, config.horizon, config.sample_step)
        original_length = len(path)
        ix, iy, inside = space.indices(path)
        clearance = np.zeros(len(path))
        clearance[inside] = space.clearance[iy[inside], ix[inside]]
        # Adjacent path samples can be half a step closer to a cell boundary.
        valid = clearance >= config.radius+config.sample_step/2
        bad = np.flatnonzero(~valid)
        end = int(bad[0]) if len(bad) else len(path)
        path, clearance = path[:end], clearance[:end]
        if len(path) < 2 or path[-1] @ route < config.min_progress:
            continue
        length = (len(path)-1)*config.horizon/(original_length-1)
        direction = math.atan2(path[-1, 1], path[-1, 0])
        if abs(direction-route_heading) > math.radians(45):
            continue
        tracking_weight = .8/(1+(position_sigma/.25)**2)
        lateral = path[-1, 1]*route[0]-path[-1, 0]*route[1]
        score = (1.3*float(path[-1] @ route)
                 + 1.8*float(np.mean(np.minimum(clearance, 1.2)))
                 - 1.2*(direction-route_heading)**2
                 - tracking_weight*(lateral-lateral_target)**2
                 - .15*(curvature-previous_curvature)**2)
        candidates.append((score, float(curvature), path, clearance, length))
    if not candidates:
        return dict(stop, reason=space.reason if space.reason != 'ok' else 'no_supported_corridor')
    _, curvature, path, clearance, length = max(candidates, key=lambda x: x[0])
    # Stop before the verified centre-path ends, allowing for observation latency.
    brake_length = max(0., length-.2)
    delay = config.reaction_time+observation_age
    speed_limit = math.sqrt((config.deceleration*delay)**2+2*config.deceleration*brake_length)-config.deceleration*delay
    speed = min(config.max_speed, speed_limit,
                config.max_yaw_rate/max(abs(curvature), 1e-6))
    return dict(status='candidate', reason='observed_support', curvature=curvature,
                speed=float(speed), yaw_rate=float(speed*curvature), path=path.tolist(),
                clearance_m=float(clearance.min()), valid_candidates=len(candidates))
