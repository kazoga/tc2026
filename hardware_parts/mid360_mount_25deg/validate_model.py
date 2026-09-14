"""出力済みSTL、STEP、DXFの製造上重要な寸法・形状を確認する。"""
from pathlib import Path
import cadquery as cq
import ezdxf
import numpy as np
import trimesh


def validate():
    output = Path(__file__).parent / 'exports'
    mesh = trimesh.load(output / 'bracket_25deg.stl', force='mesh')
    assert mesh.is_watertight and mesh.is_winding_consistent
    assert len(mesh.split()) == 1
    np.testing.assert_allclose(mesh.extents, [190, 48.940621, 41.411548], atol=.01)
    assert abs(mesh.bounds[0, 2]) < .001
    solid = cq.importers.importStep(str(output / 'bracket_25deg.step'))
    assert solid.val().isValid() and len(solid.solids().vals()) == 1
    np.testing.assert_allclose(mesh.volume, solid.val().Volume(), rtol=.001)
    assembly = cq.importers.importStep(str(output / 'assembly_reference.step')).val()
    for x in (-85, 85):
        # ねじ頭上の鉛直挿入経路。直径16mm・高さ150mmの工具包絡。
        tool = cq.Workplane('XY', origin=(x, 0, 8.01)).circle(8).extrude(150).val()
        assert assembly.intersect(tool).Volume() < 1e-5
    # ベース穴はTナット固定に必須。貫通していることを点の内外で確認。
    for x in (-85, 85):
        assert not solid.val().isInside(cq.Vector(x, 0, 4))
        assert solid.val().isInside(cq.Vector(x + 4, 0, 4))
    plate = cq.importers.importStep(str(output / 'misumi_jtabs_reference.step')).val()
    assert plate.isValid()
    bounds = plate.BoundingBox()
    np.testing.assert_allclose([bounds.xlen, bounds.ylen, bounds.zlen], [150, 60, 2], atol=.001)
    expected_area = 150*60 - 4*10*10/2 - 2*np.pi*2.75**2 - 4*np.pi*1.75**2
    np.testing.assert_allclose(plate.Volume(), expected_area*2, atol=.001)
    # C10が4隅に存在することを体積だけでなく位置でも確認。
    for x in (-74, 74):
        for y in (-29, 29):
            assert not plate.isInside(cq.Vector(x, y, 1))
    def tilt(shape):
        return shape.rotate((0, 0, 0), (1, 0, 0), -25).translate((0, 0, 30))
    assert solid.val().intersect(tilt(plate)).Volume() < 1e-5
    for x in (-50, 50):
        # 取付ねじが傾斜板と上桟を同軸で貫通する。
        shaft = cq.Workplane('XY', origin=(x, 0, -15)).circle(2.5).extrude(18).val()
        assert solid.val().intersect(tilt(shaft)).Volume() < 1e-5
        assert plate.intersect(shaft).Volume() < 1e-5
        # M5ナット・外径10mmワッシャー用の余裕。上桟下面は局所Z=-8。
        nut_space = cq.Workplane('XY', origin=(x, 0, -15)).circle(5).extrude(6.9).val()
        assert solid.val().intersect(tilt(nut_space)).Volume() < 1e-5
    doc = ezdxf.readfile(output / 'misumi_jtabs_reference.dxf')
    circles = list(doc.modelspace().query('CIRCLE'))
    assert len(circles) == 6
    assert len(list(doc.modelspace().query('LINE'))) == 8
    expected = [(x, y, radius) for xs, ys, radius in
                [((-50, 50), (0,), 2.75), ((-18, 18), (-24, 24), 1.75)]
                for x in xs for y in ys]
    actual = [(c.dxf.center.x, c.dxf.center.y, c.dxf.radius) for c in circles]
    np.testing.assert_allclose(sorted(actual), sorted(expected), atol=1e-6)
    print('PASS: 単一閉メッシュ、STEP有効性、外形、体積、M5貫通穴、板の6穴・C10、傾斜板との非干渉、ナット空間、フレーム締付工具経路')


if __name__ == '__main__':
    validate()
