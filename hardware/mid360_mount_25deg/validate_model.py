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
    np.testing.assert_allclose(mesh.extents, [154, 108.756935, 77.357951], atol=.01)
    assert abs(mesh.bounds[0, 2]) < .001
    solid = cq.importers.importStep(str(output / 'bracket_25deg.step'))
    assert solid.val().isValid() and len(solid.solids().vals()) == 1
    np.testing.assert_allclose(mesh.volume, solid.val().Volume(), rtol=.001)
    # ベース穴はTナット固定に必須。貫通していることを点の内外で確認。
    for x in (-67, 67):
        assert not solid.val().isInside(cq.Vector(x, 0, 4))
        assert solid.val().isInside(cq.Vector(x + 4, 0, 4))
    doc = ezdxf.readfile(output / 'aluminum_plate_drill.dxf')
    circles = list(doc.modelspace().query('CIRCLE'))
    assert len(circles) == 8
    expected = [(x, y, radius) for xs, ys, radius in
                [((-50, 50), (-34, 34), 2.25), ((-18, 18), (-24, 24), 1.7)]
                for x in xs for y in ys]
    actual = [(c.dxf.center.x, c.dxf.center.y, c.dxf.radius) for c in circles]
    np.testing.assert_allclose(sorted(actual), sorted(expected), atol=1e-6)
    print('PASS: 単一閉メッシュ、STEP有効性、外形、体積、M5貫通穴、板の8穴位置・径')


if __name__ == '__main__':
    validate()
