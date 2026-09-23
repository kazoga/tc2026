#!/usr/bin/env python3
"""Offline threshold sensitivity, not a pavement-accuracy benchmark.

Uses the fixed observations extracted by evaluate_free_space.replay. Never
publishes motion, fills unknown space, calls a cloud model or changes ROS params.
"""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from route_survey.free_space_core import (
    DROP, GROUND, OBSTACLE, GroundThresholds, SpaceConfig, build_space, choose_path,
)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def configurations():
    base = GroundThresholds()
    changes = {
        'baseline': {},
        'intensity_ratio_1.5': dict(material_ratio=1.5),
        'intensity_ratio_3.5': dict(material_ratio=3.5),
        'intensity_ratio_5.0': dict(material_ratio=5.),
        'rough_height_8cm': dict(rough_height=.08),
        'rough_height_10cm': dict(rough_height=.10),
        'rough_points_6': dict(rough_points=6),
        'support_points_2': dict(support_points=2),
        'support_height_4cm': dict(support_height=.04),
        'support_height_6cm': dict(support_height=.06),
        'material_points_6': dict(material_points=6),
        'material_fraction_10pct': dict(material_fraction=.1),
        'material_fraction_20pct': dict(material_fraction=.2),
        'ratio3.5_rough8cm': dict(material_ratio=3.5, rough_height=.08),
    }
    return {key: replace(base, **value) for key, value in changes.items()}


def truth_cases(thresholds):
    """Analytical counterexamples. These are illustrative, not field frequency."""
    config = SpaceConfig()
    xx, yy = np.meshgrid(np.arange(-1.175, 5.1, .05), np.arange(-3.575, 3.6, .05))
    p = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -.15),
                         np.full(xx.size, 15.)])
    side = p[:, 1] > 1.2  # Outside the sidewalk, away from the plane seed.
    probes = np.array([[1.575, 1.575], [1.575, .075]])
    result = {}
    for name, intensity in [('flat_grass_I20', 20.), ('flat_grass_I45', 45.)]:
        cloud = p.copy(); cloud[side, 3] = intensity
        space = build_space(cloud, np.empty((0, 2)), thresholds=thresholds)
        ix, iy, _ = space.indices(probes)
        result[name+'_accepted'] = bool(space.labels[iy[0], ix[0]] == GROUND)
    # A mixed cell contains earlier flat ground and current 9 cm raised returns.
    cell = (p[:, 0] >= 1.5) & (p[:, 0] < 1.65) & (p[:, 1] >= 0) & (p[:, 1] < .15)
    raised = p[cell].copy(); raised[:, 2] += .09
    space = build_space(np.vstack([p, raised]), np.empty((0, 2)), thresholds=thresholds)
    ix, iy, _ = space.indices(probes)
    result['mixed_9cm_step_accepted'] = bool(space.labels[iy[1], ix[1]] == GROUND)
    # Three real boundary returns among 36 supported samples in a 15 cm cell.
    # Rejecting sparse material evidence can erase a small piece of true grass.
    cx, cy = np.meshgrid(np.arange(1.5125, 1.65, .025), np.arange(.0125, .15, .025))
    mixed = np.column_stack([cx.ravel(), cy.ravel(), np.full(cx.size, -.15),
                              np.full(cx.size, 15.)])
    mixed[:3, 3] = 45.
    mixed_space = build_space(np.vstack([p[~cell], mixed]), np.empty((0, 2)), thresholds=thresholds)
    result['sparse_material_boundary_accepted'] = bool(mixed_space.labels[iy[1], ix[1]] == GROUND)
    lowered = p[cell].copy(); lowered[:, 2] -= .15
    drop = build_space(np.vstack([p, lowered]), np.empty((0, 2)), thresholds=thresholds)
    result['drop_preserved'] = bool(drop.labels[iy[1], ix[1]] == DROP)
    obstacle = build_space(p, probes[1:], thresholds=thresholds)
    result['obstacle_preserved'] = bool(obstacle.labels[iy[1], ix[1]] == OBSTACLE)
    missing = build_space(p[~cell], np.empty((0, 2)), thresholds=thresholds)
    result['unobserved_preserved'] = bool(missing.labels[iy[1], ix[1]] == 0)
    return result


def evaluate(scene_dir, output):
    scenes = json.loads((scene_dir/'scenes.json').read_text())
    configs = configurations()
    stats = {key: dict(thresholds=asdict(value), moving_total=0, candidates=0,
                      gained=0, lost=0, gnss_off_total=0, gnss_off_candidates=0,
                      first_half=[0, 0], second_half=[0, 0],
                      obstacle_label_changes=0, drop_label_changes=0,
                      min_raw_obstacle_distance_m=None,
                      counterexamples=truth_cases(value)) for key, value in configs.items()}
    records = []
    baseline_mismatches = 0
    for n, scene in enumerate(scenes):
        with np.load(scene_dir/scene['scene_file']) as saved:
            ground, obstacles = saved['ground'], saved['obstacles']
        record = dict(t=scene['t'], moving=scene['moving'], running=scene['running'], results={})
        baseline = build_space(ground, obstacles)
        for name, threshold in configs.items():
            space = baseline if name == 'baseline' else build_space(ground, obstacles, thresholds=threshold)
            choice = choose_path(space, math.radians(scene['route_heading_deg']),
                                 scene['lateral_target_m'],
                                 previous_curvature=scene['previous_curvature'],
                                 observation_age=scene['observation_age'])
            if name == 'baseline' and (choice['status'] != scene['decision']['status']
                                      or abs(choice['curvature']-scene['decision']['curvature']) > 1e-10):
                baseline_mismatches += 1
            record['results'][name] = dict(status=choice['status'], reason=choice['reason'],
                                           curvature=choice['curvature'], path=choice['path'],
                                           ground_cells=int(np.sum(space.labels == GROUND)))
            if not (scene['moving'] and scene['running']):
                continue
            s = stats[name]; candidate = choice['status'] == 'candidate'
            original = scene['decision']['status'] == 'candidate'
            s['moving_total'] += 1; s['candidates'] += candidate
            s['gained'] += candidate and not original
            s['lost'] += original and not candidate
            s['gnss_off_total'] += scene['gnss_off']
            s['gnss_off_candidates'] += scene['gnss_off'] and candidate
            half = s['first_half' if scene['t'] < 120. else 'second_half']
            half[0] += candidate; half[1] += 1
            s['obstacle_label_changes'] += int(np.sum((baseline.labels == OBSTACLE) & (space.labels != OBSTACLE)))
            s['drop_label_changes'] += int(np.sum((baseline.labels == DROP) & (space.labels != DROP)))
            if candidate and len(obstacles):
                distance = float(cKDTree(obstacles).query(np.asarray(choice['path']))[0].min())
                s['min_raw_obstacle_distance_m'] = min(s['min_raw_obstacle_distance_m'] or math.inf, distance)
        records.append(record)
        if (n+1) % 40 == 0:
            print('swept', n+1, 'snapshots', flush=True)
    summary = dict(baseline_mismatches=baseline_mismatches, configurations=stats,
                   note='Candidate availability, NOT semantic accuracy. Same recorded inputs and baseline previous curvature. No rollout. Retrospective gravity calibration.')
    write_json(output/'summary.json', summary)
    write_json(output/'decisions.json', records)
    if baseline_mismatches:
        raise ValueError('Baseline replay did not reproduce; do not interpret the sweep')
    return summary


def plot(summary, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    stats = summary['configurations']
    labels = list(stats)
    values = [stats[k]['candidates'] for k in labels]
    fig, ax = plt.subplots(figsize=(9, 6.5), constrained_layout=True)
    bars = ax.barh(labels[::-1], values[::-1], color='#366e92')
    ax.axvline(stats['baseline']['candidates'], color='#cd6937', linestyle='--')
    for bar in bars:
        ax.text(bar.get_width()+1, bar.get_y()+bar.get_height()/2, str(int(bar.get_width())), va='center')
    total = stats['baseline']['moving_total']
    ax.set(xlim=(0, total), xlabel=f'Snapshots with a path candidate / {total} moving snapshots',
           title='Threshold sensitivity on the same recorded observations\nMore candidates does NOT establish more accurate sidewalk detection')
    fig.savefig(output/'threshold_comparison.png', dpi=160)
    plt.close(fig)


def example_plot(replay_dir, scene_dir, output, requested=73):
    """Camera is visual context only; no pixel-to-LiDAR correspondence implied."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.font_manager import FontProperties
    font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if not font_path.exists():
        return
    plt.rcParams['font.family'] = FontProperties(fname=font_path).get_name()
    plt.rcParams['font.size'] = 12
    scenes = json.loads((scene_dir/'scenes.json').read_text())
    scene = min(scenes, key=lambda v: abs(v['t']-requested))
    with np.load(scene_dir/scene['scene_file']) as saved:
        ground, obstacles = saved['ground'], saved['obstacles']
    config = SpaceConfig()
    fig = plt.figure(figsize=(8, 10), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=[.85, 1.])
    camera = fig.add_subplot(grid[0, :])
    camera.imshow(plt.imread(replay_dir/f'camera_{requested}.jpg'))
    camera.set_title(f'約{requested}秒：カメラ画像（位置合わせ未校正・目視参照）')
    camera.axis('off')
    colors = ListedColormap(['#dde2e6', '#a2d4ad', '#bf9f5d', '#cd4852', '#956db4'])
    for column, (ratio, title) in enumerate([(2.5, '現状：反射強度の倍率 2.5'),
                                          (5., '緩和：反射強度の倍率 5.0')]):
        space = build_space(ground, obstacles, thresholds=GroundThresholds(material_ratio=ratio))
        decision = choose_path(space, math.radians(scene['route_heading_deg']), scene['lateral_target_m'],
                               previous_curvature=scene['previous_curvature'],
                               observation_age=scene['observation_age'])
        ax = fig.add_subplot(grid[1, column])
        ax.imshow(space.labels.T, origin='lower', extent=[config.ymin, config.ymax, config.xmin, config.xmax],
                  cmap=colors, vmin=0, vmax=4, interpolation='nearest')
        ax.invert_xaxis()
        ax.add_patch(plt.Circle((0, 0), config.radius, fill=False, color='#153d5c', lw=2))
        if decision['path']:
            path = np.asarray(decision['path']); ax.plot(path[:, 1], path[:, 0], 'k-', lw=2)
        ax.set(xlabel='左方向 [m]', ylabel='前方 [m]',
               title=title+'\n'+('候補あり' if decision['status'] == 'candidate' else '停止'))
    fig.suptitle('閾値を緩めると緑の領域が拡大する\n緑=路面候補 / 茶=材質・凹凸で除外 / 赤=障害物\n灰=未観測 / 紫=落ち込み / 黒線=進路候補', fontsize=13)
    fig.savefig(output/f'threshold_scene_{requested}.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay-dir', type=Path, required=True,
                        help='evaluate_free_space output containing cache/manifest.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scenes = args.output/'scenes'
    if not (scenes/'scenes.json').exists():
        from evaluate_free_space import replay
        info = json.loads((args.replay_dir/'cache/manifest.json').read_text())
        with np.load(args.replay_dir/'cache/telemetry.npz') as saved:
            data = dict(saved)
        replay(args.replay_dir, info, data, scene_output=scenes)
    summary = evaluate(scenes, args.output)
    plot(summary, args.output)
    example_plot(args.replay_dir, scenes, args.output)
    for name, result in summary['configurations'].items():
        print(name, result['candidates'], '/', result['moving_total'], flush=True)


if __name__ == '__main__':
    main()
