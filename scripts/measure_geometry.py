#!/usr/bin/env python3
"""Re-measure the Mochi geometry from the STL files and compare with hexapod_cad.py and mochi.json.

Prints each measurement and exits non-zero if one drifts by more than 0.1 mm.
"""
import json, sys, pathlib
import numpy as np, trimesh
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import hexapod_cad as cad

JSON_PATH = cad.MOCHI.parents[2] / 'software/path_tool/robots/mochi.json'
fails = []

def fit(p):
    A = np.c_[2 * p, np.ones(len(p))]
    s = np.linalg.lstsq(A, (p ** 2).sum(1), rcond=None)[0]
    c = s[:2]
    r = np.sqrt(s[2] + c @ c)
    return c, r, np.abs(np.linalg.norm(p - c, axis=1) - r).max()

def circles(m, ax, t, dmin, dmax):
    oth = [i for i in range(3) if i != ax]
    n = np.zeros(3); n[ax] = 1; o = np.zeros(3); o[ax] = t
    out = []
    for d in m.section(plane_origin=o, plane_normal=n).discrete:
        if len(d) >= 8:
            c, r, e = fit(d[:, oth])
            if e < 0.08 and dmin <= 2 * r <= dmax:
                out.append((c, 2 * r))
    return out

def check(label, value, expected, tol=0.1):
    ok = abs(value - expected) <= tol
    print(f"{'ok  ' if ok else 'FAIL'} {label:48s} measured {value:8.2f}  model {expected:8.2f}")
    if not ok:
        fails.append(label)

L = lambda r: trimesh.load_mesh(cad.MOCHI / r, force='mesh')
base = L('body/body_base.STL')
seats = circles(base, 1, 2.0, 6.9, 7.1)
ctr = (base.bounds[0] + base.bounds[1])[[0, 2]] / 2
radii = [np.linalg.norm(c - ctr) for c, _ in seats]
angles = sorted(round(float(np.degrees(np.arctan2(*(c - ctr)[::-1])))) for c, _ in seats)
print(f"body_base bbox {np.round(base.extents, 2)} mm, seats found {len(seats)}, angles {angles}")
assert len(seats) == 6
check('hip radius, mean of 6 bearing seats', float(np.mean(radii)), cad.HIP_RADIUS)
print(f"     seat radius spread {max(radii) - min(radii):.3f} mm, diameter {seats[0][1]:.2f} mm")

jb = L('joint/joint_bottom.STL')
pin = circles(jb, 2, 2.0, 4.0, 4.4)[0][0]
screw = [c for c, d in circles(jb, 2, 2.0, 1.8, 2.2)][0]
check('coxa: joint_bottom pivot to cross hole x 2', 2 * abs(pin[0] - screw[0]), cad.COXA)
cross = L('joint/joint_cross.STL')
check('joint_cross half extent', float(cross.extents[0]) / 2, cad.CROSS_HALF)

lb = L('leg/leg_bottom.STL')
b = sorted(c[0] for c, d in circles(lb, 1, 2.0, 6.9, 7.1))
check('femur: leg_bottom bearing seat spacing', b[1] - b[0], cad.FEMUR)

ft = L('foot/foot_top.STL')
holes = sorted(c[0] for c, d in circles(ft, 1, 1.5, 1.8, 2.2))
pivot = (holes[0] + holes[1]) / 2
check('foot_top: pivot to far ground screw', holes[3] - pivot, 54.0)
check('foot_top: pivot to near ground screw', holes[2] - pivot, 26.0)
g = L('foot/foot_ground.STL')
sec = g.section(plane_origin=[0, 0, 2.0], plane_normal=[0, 0, 1])
outer = max(sec.discrete, key=len)[:, :2]
dy = np.abs(outer[:, 1] - 19.75)
# The 2 mm screw channel runs along the centre line and opens into the 9 mm notch at the notch floor.
floor = float(outer[(outer[:, 0] > 54) & (dy <= 1.25), 0].max())
tip = L('foot/foot_tip.STL')
web = tip.section(plane_origin=[0, 0, 6.0], plane_normal=[0, 0, 1])
wx = np.vstack([d[:, 0][:, None] for d in web.discrete])
beyond = float(wx.max() - wx.min())
print(f"     foot_ground notch floor x = {floor:.2f}, foot_tip beyond floor = {beyond:.2f}")
check('tibia: 20 + notch floor + tip protrusion', 20.0 + floor + beyond, cad.TIBIA)
check('arm height: base + servo side + top', 5.0 + L('body/body_servo_side.STL').extents[2] + L('body/body_top.STL').extents[2], cad.ARM_HEIGHT)
check('foot_ground width', float(g.extents[1]), cad.FOOT_WIDTH)
head = L('body/body_head.STL')
print(f"body_head bbox {np.round(head.extents, 2)}, body_top bbox {np.round(L('body/body_top.STL').extents, 2)}, body height {cad.BODY_HEIGHT:.1f}")

cfg = json.loads(JSON_PATH.read_text())
print('\nCross-check with the published software/path_tool/robots/mochi.json:')
for label, pub, model in (('coxa', cfg['legJoint1ToJoint2'], cad.COXA), ('femur', cfg['legJoint2ToJoint3'], cad.FEMUR), ('tibia', cfg['legJoint3ToTip'], cad.TIBIA)):
    print(f"  {label:6s} published {pub:7.2f}  STL {model:7.2f}  diff {model - pub:+.2f} mm")
print(f"  hip R  published {np.hypot(cfg['legMountX'][1], cfg['legMountY'][1]):7.2f}  STL {cad.HIP_RADIUS:7.2f}")
print(f"  firmware limits (declared, not read by the IK): {cfg['jointLimits']}; standby {cfg['standbyPosture']}; laydown {cfg['laydownPosture']}")
sys.exit(1 if fails else 0)
