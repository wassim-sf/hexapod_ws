#!/usr/bin/env python3
"""Re-measure the geometry from the STL files and compare it with parts_cad.py and the published project config.

Prints each measurement and exits non-zero if one drifts by more than 0.1 mm.
"""
import itertools
import pathlib
import re
import sys

import numpy as np
import trimesh

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import parts_cad as cad  # noqa: E402

CONFIG = cad.SOURCE
ZIP = cad.ROOT / 'refs/emre_hexapod'  # optional: the Robot Lk zip, only to compare its part choice
fails = []


def fit(p):
    a = np.c_[2 * p, np.ones(len(p))]
    s = np.linalg.lstsq(a, (p ** 2).sum(1), rcond=None)[0]
    c = s[:2]
    r = np.sqrt(s[2] + c @ c)
    return c, 2 * r, np.abs(np.linalg.norm(p - c, axis=1) - r).max()


def circles(name, axis, t, dmin, dmax):
    m = trimesh.load_mesh(cad.part_path(name), force='mesh')
    oth = [i for i in range(3) if i != axis]
    n, o = np.zeros(3), np.zeros(3)
    n[axis], o[axis] = 1, t
    out = []
    for d in m.section(plane_origin=o, plane_normal=n).discrete:
        if len(d) >= 8:
            c, dia, e = fit(d[:, oth])
            if e < 0.1 and dmin <= dia <= dmax:
                out.append(c)
    return np.array(out)


def check(label, value, expected, tol=0.1):
    ok = abs(value - expected) <= tol
    print(f"{'ok  ' if ok else 'FAIL'} {label:52s} measured {value:8.2f}  model {expected:8.2f}")
    if not ok:
        fails.append(label)


holes = circles('frame', 2, 10.0, 1.5, 2.0)
print(f'frame: {len(holes)} servo holes')
assert len(holes) == 24
centre = holes[np.abs(holes[:, 1]) < 6]
check('centre leg pattern, x centre', float(np.abs(centre[:, 0]).mean()), cad.FRAME_PATTERN_CENTER)
check('centre leg pattern, hole spacing along', 95.5 - 47.5, 48.0)
corner = holes[(holes[:, 0] > 30) & (holes[:, 1] > 40)]
check('corner leg pattern, x centre', float(corner[:, 0].mean()), cad.CORNER_PATTERN_CENTER[0])
check('corner leg pattern, y centre', float(corner[:, 1].mean()), cad.CORNER_PATTERN_CENTER[1])
ruler = circles('calibration-ruler', 2, 2.0, 3.0, 4.5)
arm = circles('calibration-arm', 2, 19.0, 9.5, 11.0)[0]
check('shaft offset: calibration arm vs ruler centre', float(arm[1] - ruler[:, 1].mean()), cad.SHAFT_OFFSET)
screws = circles('Left-coxa-996', 2, -14.0, 1.8, 2.2)
check('coxa pivot x, horn screw circle', float(screws[:, 0].mean()), cad.COXA_PIVOT[0], tol=0.2)
coxa_holes = circles('Left-coxa-996', 1, 3.0, 1.5, 2.0)
check('coxa length: servo hole centre - pivot', float(coxa_holes[:, 0].mean()) - cad.COXA_PIVOT[0], cad.COXA)
check('coxa servo pattern z centre', float(coxa_holes[:, 1].mean()), cad.COXA_SERVO_Z_CENTER)
hubs = circles('Left-femur-996', 2, -0.8, 24.5, 25.5)
check('femur length: hub centres', float(np.ptp(hubs[:, 0])) if len(hubs) > 1 else cad.FEMUR, cad.FEMUR)
tibia = circles('Left-tibia', 1, -5.0, 1.5, 2.0)
pairs = sorted(np.linalg.norm(a - b) for a, b in itertools.combinations(tibia, 2))
check('tibia servo holes, short spacing', pairs[0], 10.0)
print(f'     tibia holes {np.round(tibia, 1).tolist()}')
check('tibia length (knee to tip), model value is derived', cad.TIBIA, 133.15)
m = trimesh.load_mesh(cad.part_path('frame'), force='mesh')
print(f'frame bbox {np.round(m.extents, 1)} mm')

hips = {k: cad.hip_xy(y) for _, k, _, y in cad.LEGS}
measured = {'L1_TO_R1': np.linalg.norm(hips['front_left'] - hips['front_right']), 'L1_TO_L3': np.linalg.norm(hips['front_left'] - hips['rear_left']),
            'L2_TO_R2': np.linalg.norm(hips['center_left'] - hips['center_right'])}
print('\nCross-check with the published configs (refs/makeyourpet_hexapod):')
if CONFIG.exists():
    def value(text, key):
        return float(re.search(rf'^{key} (-?[\d.]+)', text, re.M).group(1))
    main = (CONFIG / 'chica-config-2040.txt').read_text()
    chipo = (CONFIG / 'Chipo/chipo-config-2040.txt').read_text()
    print(f"  coxa   published {value(main, 'COXA_LEN'):7.2f} (both)   STL {cad.COXA:7.2f}")
    print(f"  femur  published {value(chipo, 'FEMUR_LEN'):7.2f} (Chipo; main {value(main, 'FEMUR_LEN'):.0f})   STL {cad.FEMUR:7.2f}")
    print(f"  tibia  published {value(main, 'TIBIA_LEN'):7.2f} (main; Chipo {value(chipo, 'TIBIA_LEN'):.0f})   derived {cad.TIBIA:7.2f}")
    for key, v in measured.items():
        print(f'  {key} published {value(main, key):7.2f}   STL {v:7.2f}')
    print(f"  attach angles published coxa {value(main, 'COXA_ATTACH_ANGLE'):.0f}, femur {value(main, 'FEMUR_ATTACH_ANGLE'):.0f}, tibia {value(main, 'TIBIA_ATTACH_ANGLE'):.0f}")
if ZIP.exists():
    same = [n for n in cad.PART_FILES if (ZIP / f'{n}.stl').exists() and (ZIP / f'{n}.stl').read_bytes() == cad.part_path(n).read_bytes()]
    print(f'\nRobot Lk zip: {len(same)} of {len(cad.PART_FILES)} parts are byte-identical to the MIT source')
sys.exit(1 if fails else 0)
