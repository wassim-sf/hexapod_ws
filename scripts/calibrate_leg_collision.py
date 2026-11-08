#!/usr/bin/env python3
"""Fit the leg_collision capsule radii against true mesh-to-mesh contact between neighbouring legs.

Truth comes from FCL on the full-resolution printed parts plus MG92B body boxes (hexapod_cad, as in
sweep_limits.py). The capsule model is what hexapod_kinematics uses at runtime. Needs refs/ and .venv.
"""
import pathlib
import sys

import numpy as np
import trimesh
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import hexapod_cad as cad  # noqa: E402
from hexapod_kinematics import RobotModel, link_frames  # noqa: E402
from hexapod_kinematics.checks import _segment_distance  # noqa: E402

PARAMS = pathlib.Path(__file__).resolve().parents[1] / 'src/hexapod_description/config/robot_parameters.yaml'
data = yaml.safe_load(PARAMS.read_text())
SPLIT = 0.060  # m: where the foot side plates end, from foot_top and foot_bottom (60 mm past the pivot)
data['leg_collision'] = {'femur_radius': 0.0, 'tibia_upper_radius': 0.0, 'tibia_lower_radius': 0.0, 'tibia_split': SPLIT}
model = RobotModel(data, PARAMS)
rng = np.random.default_rng(11)


def servo_boxes():
    out = []
    for x in (5.3, 38.3):
        box = trimesh.creation.box(extents=(22.8, 25, 12))
        box.apply_translation((x, 3.7, 0))
        out.append(box)
    return out


def link_mesh(parts, extra=(), side='right', translate=None):
    meshes = [m for _, _, m in cad.placed(parts)] + list(extra)
    merged = trimesh.util.concatenate(meshes)
    if translate is not None:
        merged.apply_translation(translate)
    merged.apply_scale(0.001)
    return cad.mirror(merged) if side == 'left' else merged


class MeshPair:
    def __init__(self, i, j):
        self.i, self.j = i, j
        self.legs = (model.legs[i], model.legs[j])
        self.managers = []
        for leg in self.legs:
            manager = trimesh.collision.CollisionManager()
            manager.add_object('coxa', link_mesh(cad.COXA_PARTS, side=leg.side))
            manager.add_object('femur', link_mesh(cad.FEMUR_PARTS, servo_boxes(), leg.side))
            manager.add_object('tibia', link_mesh(cad.TIBIA_PARTS, side=leg.side))
            manager.add_object('foot', link_mesh(cad.FOOT_PARTS, side=leg.side))
            self.managers.append(manager)

    def collide(self, qa, qb):
        for manager, leg, q in zip(self.managers, self.legs, (qa, qb)):
            frames = link_frames(model, leg.index, q)
            for name in ('coxa', 'femur', 'tibia', 'foot'):
                manager.set_transform(name, frames[name])
        return self.managers[0].in_collision_other(self.managers[1])


def capsule_distances(i, j, qa, qb):
    """Axis distance for each capsule pair, keyed by ('femur' | 'upper' | 'lower') names."""
    segments = []
    for index, q in ((i, qa), (j, qb)):
        frames = link_frames(model, index, q)
        def point(frame, xyz):
            return (frames[frame] @ np.array([*xyz, 1.0]))[:3]
        segments.append({
            'femur': (point('femur', (0, 0, 0)), point('femur', (model.femur, 0, 0))),
            'upper': (point('tibia', (0, 0, 0)), point('tibia', (0, 0, -SPLIT))),
            'lower': (point('tibia', (0, 0, -SPLIT)), point('tibia', (0, 0, -model.tibia))),
        })
    return {(a, b): _segment_distance(*segments[0][a], *segments[1][b]) for a in segments[0] for b in segments[1]}


def toward(i, j, angle):
    """Coxa angles that swing legs i and j toward each other by angle."""
    yi, yj = model.legs[i].yaw, model.legs[j].yaw
    wrap = (yj - yi + np.pi) % (2 * np.pi) - np.pi
    sign = np.sign(wrap)
    return sign * angle, -sign * angle


def contact_angle(check, i, j, femur, tibia):
    for degrees in range(0, 91):
        a, b = toward(i, j, np.radians(degrees))
        if check((a, femur, tibia), (b, femur, tibia)):
            return degrees
    return None


PAIRS = {'right side (1,2)': (0, 1), 'front (1,4)': (0, 3), 'rear (3,6)': (2, 5), 'left side (4,5)': (3, 4)}
POSES = [(0, 0), (30, -15), (65, 0)]
mesh_pairs = {name: MeshPair(*pair) for name, pair in PAIRS.items()}

print('First mesh contact, degrees of coxa swing per leg toward the neighbour:')
truth_angle = {}
for name, (i, j) in PAIRS.items():
    for f, t in POSES:
        truth_angle[(name, f, t)] = contact_angle(mesh_pairs[name].collide, i, j, np.radians(f), np.radians(t))
        print(f'  {name:18s} femur {f:3d} tibia {t:4d}: {truth_angle[(name, f, t)]}')

# Random poses, biased toward the legs being close, because most poses are nowhere near contact.
N = 8000
names = list(PAIRS)
records = []
for n in range(N):
    name = names[n % len(names)]
    i, j = PAIRS[name]
    swing = rng.uniform(0.3, 1.0) * model.upper[0]
    a, b = toward(i, j, swing)
    qa = np.array([a + rng.normal(0, 0.1), rng.uniform(model.lower[1], model.upper[1]), rng.uniform(model.lower[2], model.upper[2])])
    qb = np.array([b + rng.normal(0, 0.1), rng.uniform(model.lower[1], model.upper[1]), rng.uniform(model.lower[2], model.upper[2])])
    qa[0], qb[0] = np.clip(qa[0], model.lower[0], model.upper[0]), np.clip(qb[0], model.lower[0], model.upper[0])
    records.append((mesh_pairs[name].collide(qa, qb), capsule_distances(i, j, qa, qb)))
truth = np.array([r[0] for r in records])
fit = np.arange(N) % 2 == 0  # alternate samples, so both halves cover all pair types
print(f'\n{N} random close poses: {int(truth.sum())} in true contact; half used to fit, half held out')


def predict(rf, ru, rl):
    radius = {'femur': rf, 'upper': ru, 'lower': rl}
    keys = list(records[0][1])
    return np.array([any(r[1][k] - radius[k[0]] - radius[k[1]] < 0 for k in keys) for r in records])


# Femur and lower tibia rarely decide contact, so a free fit shrinks them to nothing. Fix them from the
# parts instead: femur block half-height 11.2 mm, foot tip half-width about 6 mm. Fit only the upper tibia.
rf, rl = 0.012, 0.006
best = None
for ru in np.arange(0.018, 0.0341, 0.001):
    p = predict(rf, ru, rl)
    # Prefer few missed contacts: a false alarm costs range, a miss breaks hardware.
    score = 5 * int((truth & ~p & fit).sum()) + int((~truth & p & fit).sum())
    if best is None or score < best[0]:
        best = (score, round(float(ru), 4))
_, ru = best
p = predict(rf, ru, rl)
print(f'Result: femur {rf * 1000:.0f} mm, tibia upper {ru * 1000:.0f} mm, tibia lower {rl * 1000:.0f} mm')
for label, mask in (('fit', fit), ('held out', ~fit)):
    print(f'  {label:9s}: {int((truth & ~p & mask).sum())} of {int((truth & mask).sum())} contacts missed, '
          f'{int((~truth & p & mask).sum())} of {int((~truth & mask).sum())} false alarms')
radius = {'femur': rf, 'upper': ru, 'lower': rl}
gaps = [min(v - radius[k[0]] - radius[k[1]] for k, v in records[n][1].items()) for n in np.flatnonzero(truth & ~p)]
if gaps:
    print(f'  missed contacts: capsule gap {min(gaps) * 1000:.1f} to {max(gaps) * 1000:.1f} mm when the meshes first touch')

radii = {'femur': rf, 'upper': ru, 'lower': rl}


def capsule_collide(i, j):
    def check(qa, qb):
        d = capsule_distances(i, j, np.array(qa), np.array(qb))
        return any(v - radii[k[0]] - radii[k[1]] < 0 for k, v in d.items())
    return check


print('\nContact angle at the fitted radii, capsule vs mesh:')
for name, (i, j) in PAIRS.items():
    for f, t in POSES:
        print(f'  {name:18s} femur {f:3d} tibia {t:4d}: capsule {contact_angle(capsule_collide(i, j), i, j, np.radians(f), np.radians(t))}, mesh {truth_angle[(name, f, t)]}')
