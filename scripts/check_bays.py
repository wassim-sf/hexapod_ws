#!/usr/bin/env python3
"""Test whether the electronics fit, as boxes against the real body meshes and the coxa servos."""
import pathlib
import re
import sys

import numpy as np
import trimesh

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import emre_cad as cad  # noqa: E402
import sweep_limits as sweep  # noqa: E402

src = (pathlib.Path(__file__).parent / 'generate_model.py').read_text()
block = src[src.index('FLOOR = -'):src.index('def printed_grams')]
scope = {}
exec(block, scope)
LAYOUT = [(n, s, c) for n, _, s, c in scope['ELECTRONICS'] if n != 'main_wiring_fuse']
TRAY_TOP = scope['TRAY_TOP']
BOARD = TRAY_TOP + 5.0  # Pi board on 5 mm standoffs on a printed tray above the floor boards
EXTRA = [('pi5_ports', (18.0, 52.3, 16.0), (37.5, -1.6, BOARD + 1.5 + 8.0)),  # official outline: 3 mm overhang, 16 mm tall (read off the 1:1 drawing)
         ('fan_headroom_8mm', (30.0, 30.0, 8.0), (0.0, 5.3, BOARD + 1.5 + 13.7 + 4.0))]


def boxes(layout):
    out = {}
    for name, size, centre in layout:
        b = trimesh.creation.box(extents=size)
        b.apply_translation(centre)
        b.apply_scale(0.001)
        out[name] = b
    return out


body_parts = [cad.placed(n, t) for _, n, t in cad.body_parts()] + [sweep.servo_body(cad.body_servo_t(y)) for *_, y in cad.LEGS]
body = trimesh.util.concatenate(body_parts)
body.apply_scale(0.001)
target = trimesh.collision.CollisionManager()
target.add_object('body', body)


def test(layout, label):
    items = boxes(layout)
    print(f'\n{label}')
    ok = True
    manager = trimesh.collision.CollisionManager()
    for name, mesh in items.items():
        manager.add_object(name, mesh)
        hit = target.in_collision_single(mesh)
        gap = target.min_distance_single(mesh)
        lo, hi = mesh.bounds * 1000
        print(f"  {name:18s} x {lo[0]:6.1f}..{hi[0]:6.1f}  y {lo[1]:6.1f}..{hi[1]:6.1f}  z {lo[2]:6.1f}..{hi[2]:6.1f}  {'HITS BODY' if hit else f'clear by {gap * 1000:.1f} mm'}")
        ok &= not hit
    hit, names = manager.in_collision_internal(return_names=True)
    pairs = sorted({tuple(sorted(p)) for p in names} - {('active_cooler', 'pi5'), ('fan_headroom_8mm', 'pi5'), ('active_cooler', 'fan_headroom_8mm'), ('pi5', 'pi5_ports'), ('active_cooler', 'pi5_ports')})
    print('  overlaps between items:', pairs or 'none')
    return ok and not pairs


proposed = test(LAYOUT + EXTRA, 'Proposed layout, no shell change: boards on the floor, Pi on a tray above them, LiPo and BEC under the body')
single = test([('arduino_mega', (101.6, 53.4, 15.0), (0, 0, cad.FLOOR_TOP + 7.8))], 'For scale: an Arduino Mega on the floor')
print(f"\nResult: proposed layout {'FITS' if proposed else 'DOES NOT FIT'}. Pi board at z = {BOARD:.1f} mm, cooler top {BOARD + 1.5 + 13.7:.1f} mm, ports top {BOARD + 1.5 + 16:.1f} mm; frame top 16, top cover top 28.2 mm.")
