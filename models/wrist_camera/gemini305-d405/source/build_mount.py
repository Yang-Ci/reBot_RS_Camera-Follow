"""D405 / Gemini 305 centered level-view mount modification, 2026-11-13.

Requires cadquery==2.5.2. Units: mm. Run from any working directory.
Derived from Seeed-Projects/reBot-DevArm; CERN-OHL-W-2.0.
Only the connector is rebuilt; clamp and camera cradle interfaces are preserved.
"""
import hashlib
import json
import math
from pathlib import Path
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface

SOURCE = Path(__file__).resolve().parent
OUTPUT = SOURCE.parent
CLAMP_CUT_Y = 35.5
CRADLE_CUT_Y = 44.0
PIVOT = (0.0, 44.0, -11.0)


def cutting_box(y0, y1):
    return cq.Solid.makeBox(400, y1-y0, 400, cq.Vector(-200, y0, -200))


def cut_face(shape, y):
    faces = [face for face in shape.Faces()
             if face.geomType() == 'PLANE'
             and abs(face.Center().y-y) < 1e-5
             and abs(face.normalAt().y) > .99999]
    assert len(faces) == 1 and len(faces[0].Wires()) == 1
    return faces[0]


def build():
    original = cq.importers.importStep(str(SOURCE/'original_mount.step')).val()
    seat = [face for face in original.Faces()
            if face.geomType() == 'PLANE' and face.Area() > 1500
            and face.normalAt().z > .8 and face.normalAt().y < -.1]
    assert len(seat) == 1
    normal = seat[0].normalAt()
    angle = -math.degrees(math.atan2(-normal.y, normal.z))
    # Registration datum: midpoint of the two unchanged arm-side fixing bores.
    clamp_holes, camera_holes = [], []
    for face in original.Faces():
        if face.geomType() != 'CYLINDER':
            continue
        cyl = BRepAdaptor_Surface(face.wrapped).Cylinder()
        axis = cyl.Axis()
        x = axis.Location().X()
        if abs(cyl.Radius()-1.35) < 1e-6 and abs(axis.Direction().Z()) > .999:
            if not any(abs(x-y) < 1e-6 for y in clamp_holes): clamp_holes.append(x)
        if abs(cyl.Radius()-1.8) < 1e-6 and abs(axis.Direction().Z()) > .85:
            if not any(abs(x-y) < 1e-6 for y in camera_holes): camera_holes.append(x)
    assert len(clamp_holes) == 2 and len(camera_holes) == 2
    clamp_center = sum(clamp_holes)/2
    old_camera_center = sum(camera_holes)/2
    shift = clamp_center-old_camera_center
    assert abs(old_camera_center+shift-clamp_center) < 1e-9
    axis_end = (PIVOT[0]+1, PIVOT[1], PIVOT[2])
    clamp = original.intersect(cutting_box(-100, CLAMP_CUT_Y))
    cradle = original.intersect(cutting_box(CRADLE_CUT_Y, 200))
    a = cut_face(clamp, CLAMP_CUT_Y)
    b = cut_face(cradle, CRADLE_CUT_Y).rotate(PIVOT, axis_end, angle).translate((shift, 0, 0))
    moved = cradle.rotate(PIVOT, axis_end, angle).translate((shift, 0, 0))
    bridge = cq.Solid.makeLoft([a.outerWire(), b.outerWire()], ruled=True)
    result = clamp.fuse(bridge, moved).clean()
    assert result.isValid() and len(result.Solids()) == 1
    assert clamp.cut(result).Volume() < 1e-6
    assert result.intersect(cutting_box(-100, CLAMP_CUT_Y)).cut(clamp).Volume() < 1e-6
    assert moved.cut(result).Volume() < 1e-6
    new_seat = seat[0].rotate(PIVOT, axis_end, angle).normalAt()
    assert abs(new_seat.y) < 1e-9 and abs(new_seat.z-1) < 1e-9
    # Check the camera interface after undoing its rigid motion. Comparing
    # identical coplanar faces by a second Boolean can be ill-conditioned.
    reference = original.intersect(cutting_box(46.0, 200))
    restored = result.translate((-shift, 0, 0)).rotate(PIVOT, axis_end, -angle).intersect(cutting_box(46.0, 200))
    ref_vertices = [v.Center() for v in reference.Vertices()]
    restored_vertices = [v.Center() for v in restored.Vertices()]
    assert len(ref_vertices) == len(restored_vertices)
    vertex_error = max(min((a-b).Length for b in restored_vertices) for a in ref_vertices)
    assert vertex_error < 1e-7
    assert abs(reference.Volume()-restored.Volume()) < 1e-5
    assert abs(reference.Area()-restored.Area()) < 1e-5
    assert len(reference.Faces()) == len(restored.Faces())
    return original, result, {
        'units': 'mm',
        'source_repository': 'https://github.com/Seeed-Projects/reBot-DevArm',
        'source_blob_sha': '812120494a7e6145c5971e24d4d27c8e04603090',
        'source_sha256': hashlib.sha256((SOURCE/'original_mount.step').read_bytes()).hexdigest(),
        'original_nominal_downward_pitch_deg': -angle,
        'new_nominal_downward_pitch_deg': 0.0,
        'rotation_about_CAD_X_deg': angle,
        'rotation_pivot_mm': PIVOT,
        'lateral_translation_mm': shift,
        'clamp_centerline_CAD_X_mm': clamp_center,
        'old_camera_centerline_CAD_X_mm': old_camera_center,
        'new_camera_centerline_CAD_X_mm': old_camera_center+shift,
        'lateral_centering_error_mm': abs(old_camera_center+shift-clamp_center),
        'original_volume_mm3': original.Volume(),
        'modified_volume_mm3': result.Volume(),
        'clamp_preserved': True,
        'camera_cradle_rigidly_preserved': True,
        'camera_interface_max_vertex_error_mm': vertex_error,
        'camera_interface_volume_delta_mm3': abs(reference.Volume()-restored.Volume()),
        'camera_interface_area_delta_mm2': abs(reference.Area()-restored.Area()),
        'solid_count': len(result.Solids()),
        'geometrically_valid': result.isValid(),
    }


if __name__ == '__main__':
    original, result, report = build()
    cq.exporters.export(result, str(OUTPUT/'Gemini305_D405_Mount.step'))
    cq.exporters.export(result, str(OUTPUT/'Gemini305_D405_Mount.stl'), tolerance=.05, angularTolerance=.1)
    check = cq.importers.importStep(str(OUTPUT/'Gemini305_D405_Mount.step')).val()
    assert check.isValid() and len(check.Solids()) == 1
    assert abs(check.Volume()-result.Volume()) < .001
    (OUTPUT/'geometry_check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
