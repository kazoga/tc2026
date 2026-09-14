"""MID-360用25度ブラケット。単位mm、X=横梁方向、Y=前方、Z=上。"""
from pathlib import Path
import json
import math
import argparse

import cadquery as cq


def build(output: Path) -> dict:
    angle = 25.
    output.mkdir(parents=True, exist_ok=True)
    height = 52.
    c = math.cos(math.radians(angle))
    slope = math.tan(math.radians(angle))
    # 上側の支持面: z = height - y * tan(angle)
    end = 60*c
    outer = [(-end, 0), (end, 0), (end, height-end*slope),
             (-end, height+end*slope)]
    inner_end = 46*c
    inner = [(-inner_end, 6), (inner_end, 6),
             (inner_end, height-inner_end*slope-8/c),
             (-inner_end, height+inner_end*slope-8/c)]
    base = cq.Workplane('XY').box(154, 24, 8, centered=(True, True, False))
    for x in (-50, 50):
        side = cq.Workplane('YZ', origin=(x-6, 0, 0)).polyline(outer).close().extrude(12)
        window = cq.Workplane('YZ', origin=(x-7, 0, 0)).polyline(inner).close().extrude(14)
        base = base.union(side.cut(window))
    for x in (-67, 67):
        base = base.cut(cq.Workplane('XY').center(x, 0).circle(2.75).extrude(10))

    def tilt(shape):
        return shape.rotate((0, 0, 0), (1, 0, 0), -angle).translate((0, 0, height))

    plate = cq.Workplane('XY').box(120, 140, 3, centered=(True, True, False))
    for x in (-50, 50):
        for y in (-34, 34):
            # M4はアルミ板と上桟を貫通。下面の開口からワッシャー・ナットを入れる。
            hole = cq.Workplane('XY', origin=(x, y, -20)).circle(2.25).extrude(25)
            base = base.cut(tilt(hole))
            plate = plate.cut(hole)
    # 公式底面図の48×36を回転し、ケーブルを後方に向ける。
    for x in (-18, 18):
        for y in (-24, 24):
            plate = plate.cut(cq.Workplane('XY').center(x, y).circle(1.7).extrude(4))

    # 外観・干渉検討用の簡略モデル。メーカーの詳細形状ではない。
    body = cq.Workplane('XY', origin=(0, 0, 3)).box(65, 65, 39.5, centered=(True, True, False))
    dome = cq.Workplane('XY', origin=(0, 0, 42.5)).circle(25).extrude(20.5)
    connector = cq.Workplane('XZ', origin=(0, -32.5, 17)).circle(7).extrude(8)
    sensor = tilt(body.union(dome).union(connector))
    beam = cq.Workplane('XY', origin=(0, 0, -20)).box(220, 20, 20, centered=(True, True, False))
    # 取付溝は視覚表現。HFS5断面の製造形状ではない。
    beam = beam.cut(cq.Workplane('XY', origin=(0, 0, -3)).box(221, 6, 4, centered=(True, True, False)))
    plate_tilted = tilt(plate)
    parts = dict(bracket=base.val(), aluminum_plate=plate_tilted.val(),
                 sensor_envelope=sensor.val(), frame_envelope=beam.val())
    for name, shape in parts.items():
        if not shape.isValid():
            raise RuntimeError(f'{name}: invalid solid')
    if len(base.solids().vals()) != 1:
        raise RuntimeError('印刷部品が一体につながっていない')
    cq.exporters.export(base, str(output/'bracket_25deg.stl'), tolerance=.05, angularTolerance=.1)
    cq.exporters.export(base, str(output/'bracket_25deg.step'))
    cq.exporters.export(plate, str(output/'aluminum_plate_120x140x3.step'))
    # 1:1加工図に使う2D輪郭（底面平面を直接投影し穴を保持）。
    cq.exporters.export(plate.faces('<Z').wires(), str(output/'aluminum_plate_drill.dxf'))
    assembly = cq.Assembly()
    for name, shape in parts.items():
        assembly.add(shape, name=name)
    assembly.save(str(output/'assembly_reference.step'))
    cq.exporters.export(cq.Compound.makeCompound(list(parts.values())),
                        str(output/'assembly_reference.stl'), tolerance=.1)
    from preview_model import render
    render(parts, output, angle)
    bounds = base.val().BoundingBox()
    results = dict(angle_down_deg=angle, units='mm', printer_part_count=1,
                   bracket_bounds_mm=[bounds.xlen,bounds.ylen,bounds.zlen],
                   bracket_volume_cm3=base.val().Volume()/1000,
                   base_hole_pitch_mm=134, base_hole_diameter_mm=5.5,
                   plate_size_mm=[120,140,3], sensor_hole_pattern_mm=[36,48],
                   sensor_hole_diameter_mm=3.4, plate_support_hole_pattern_mm=[100,68],
                   plate_support_hole_diameter_mm=4.5,
                   plate_exposed_top_area_estimate_mm2=120*140-65*65,
                   all_brep_valid=True, single_printed_solid=True,
                   assumed_frame='MISUMI HFS5 top slot, M5 nuts; beam top width >=20mm',
                   fit_verified_on_hardware=False, thermal_verified=False, strength_verified=False)
    (output/'dimensions.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    return parts


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'exports')
    args=parser.parse_args()
    build(args.output)
