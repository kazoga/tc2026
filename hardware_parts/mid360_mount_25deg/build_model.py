"""MID-360用25度ブラケット。単位mm、X=横梁方向、Y=前方、Z=上。"""
from pathlib import Path
import json
import math
import argparse

import cadquery as cq

PLATE_PART_NUMBER = 'JTABS-AM-A60-B150-T2-X30-G100-N5-L6-V48-W36-NA3-CC10'
FORWARD_OFFSET_MM = 50.


def existing_plate():
    """MISUMI図面のAを前後Y、Bを横Xに向け、板中心を原点にする。"""
    outline = [(-65, -30), (65, -30), (75, -20), (75, 20),
               (65, 30), (-65, 30), (-75, 20), (-75, -20)]
    plate = cq.Workplane('XY').polyline(outline).close().extrude(2)
    # N5はφ5.5。Y（図面）は省略され、(B-G)/2=25。
    for x in (-50, 50):
        plate = plate.cut(cq.Workplane('XY').center(x, 0).circle(2.75).extrude(3))
    # NA3はφ3.5。S省略=(B-W)/2=57。A方向L6とL+V54。
    for x in (-18, 18):
        for y in (-24, 24):
            plate = plate.cut(cq.Workplane('XY').center(x, y).circle(1.75).extrude(3))
    return plate


def build(output: Path) -> dict:
    angle = 25.
    output.mkdir(parents=True, exist_ok=True)
    height = 30.
    c = math.cos(math.radians(angle))
    slope = math.tan(math.radians(angle))
    # 板中心を横梁中心より50mm前へ。後端から支持面へ左右のリブを延長。
    offset = FORWARD_OFFSET_MM
    # 上側の支持面: z = height - (y-offset) * tan(angle)
    end = 27*c
    outer = [(-12, 0), (offset+end, 0), (offset+end, height-end*slope),
             (offset-end, height+end*slope), (-12, 8)]
    inner_end = 20*c
    inner = [(-inner_end, 6), (inner_end, 6),
             (inner_end, height-inner_end*slope-8/c),
             (-inner_end, height+inner_end*slope-8/c)]
    inner = [(y+offset, z) for y, z in inner]
    base = cq.Workplane('XY').box(190, 24, 8, centered=(True, True, False))
    for x in (-50, 50):
        side = cq.Workplane('YZ', origin=(x-8, 0, 0)).polyline(outer).close().extrude(16)
        window = cq.Workplane('YZ', origin=(x-9, 0, 0)).polyline(inner).close().extrude(18)
        base = base.union(side.cut(window))
    for x in (-85, 85):
        base = base.cut(cq.Workplane('XY').center(x, 0).circle(2.75).extrude(10))

    def tilt(shape):
        return shape.rotate((0, 0, 0), (1, 0, 0), -angle).translate((0, offset, height))

    plate = existing_plate()
    for x in (-50, 50):
        # 既存板のN5穴2個で固定。下の窓からM5ナットを入れる。
        hole = cq.Workplane('XY', origin=(x, 0, -16)).circle(2.75).extrude(20)
        base = base.cut(tilt(hole))

    # 外観・干渉検討用の簡略モデル。メーカーの詳細形状ではない。
    body = cq.Workplane('XY', origin=(0, 0, 2)).box(65, 65, 39.5, centered=(True, True, False))
    dome = cq.Workplane('XY', origin=(0, 0, 41.5)).circle(25).extrude(20.5)
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
    cq.exporters.export(plate, str(output/'misumi_jtabs_reference.step'))
    cq.exporters.export(plate.faces('<Z').wires(), str(output/'misumi_jtabs_reference.dxf'))
    assembly = cq.Assembly()
    for name, shape in parts.items():
        assembly.add(shape, name=name)
    assembly.export(str(output/'assembly_reference.step'))
    cq.exporters.export(cq.Compound.makeCompound(list(parts.values())),
                        str(output/'assembly_reference.stl'), tolerance=.1)
    from preview_model import render
    render(parts, output, angle, offset)
    bounds = base.val().BoundingBox()
    results = dict(angle_down_deg=angle, units='mm', printer_part_count=1,
                   design_revision=3, forward_offset_mm=offset,
                   plate_support_center_mm=[0, offset, height],
                   bracket_bounds_mm=[bounds.xlen,bounds.ylen,bounds.zlen],
                   bracket_volume_cm3=base.val().Volume()/1000,
                   base_hole_pitch_mm=170, base_hole_diameter_mm=5.5,
                   plate_part_number=PLATE_PART_NUMBER,
                   plate_size_mm=[150,60,2], plate_corner_chamfer_mm=10,
                   sensor_hole_pattern_mm=[36,48], sensor_hole_diameter_mm=3.5,
                   plate_support_hole_count=2, plate_support_hole_pitch_mm=100,
                   plate_support_hole_diameter_mm=5.5,
                   plate_exposed_top_area_estimate_mm2=150*60-4*10*10/2-65*60-2*math.pi*2.75**2,
                   manufacturer_thermal_recommendation_met=False,
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
