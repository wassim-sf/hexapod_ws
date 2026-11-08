#!/usr/bin/env python3
"""Fit the body_collision capsule radii against true mesh contact between a leg's tibia and foot and the body."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import sweep_limits as sweep  # noqa: E402
from hexapod_kinematics import body_clearances, check_joint_limits  # noqa: E402
from hexapod_kinematics.checks import _point_box_distance, body_capsules  # noqa: E402

model = sweep.model
rng = np.random.default_rng(21)
N = 6000
legs = [model.legs[i] for i in (1, 4, 0, 3, 2, 5)]
objs = {leg.index: sweep.Leg(leg, '') for leg in legs}
records = []
for n in range(N):
    leg = legs[n % len(legs)]
    q = rng.uniform(model.lower, model.upper)
    # Most random poses are far from the body, so half are drawn low and folded in.
    if rng.random() < 0.5:
        q[1] = rng.uniform(model.lower[1], 0.3)
        q[2] = rng.uniform(model.lower[2], 0.2)
    obj = objs[leg.index]
    obj.pose(q)
    names = obj.manager.in_collision_other(sweep.BODY, return_names=True)[1]
    touching = any({'tibia', 'foot'} & set(p) for p in names)
    gaps = {}
    for name, start, end, _ in body_capsules(model, leg.index, q):
        points = start + np.linspace(0.0, 1.0, 13)[:, None] * (end - start)
        gaps[name] = min(float(_point_box_distance(points, box).min()) for box in model.body_boxes)
    records.append((touching, gaps['tibia'], gaps['foot']))
truth = np.array([r[0] for r in records])
tibia = np.array([r[1] for r in records])
foot = np.array([r[2] for r in records])
fit = np.arange(N) % 4 < 2  # both halves contain both kinds of pose
print(f'{N} poses: {int(truth.sum())} in contact')
best = None
# Lower bounds are the tibia bar's half width (4.5 mm) and the tip's radius (4.5 mm).
for rt in np.arange(0.0045, 0.0301, 0.0015):
    for rf in np.arange(0.0045, 0.0201, 0.0015):
        p = (tibia < rt) | (foot < rf)
        score = 5 * int((truth & ~p & fit).sum()) + int((~truth & p & fit).sum())
        if best is None or score < best[0]:
            best = (score, round(float(rt), 4), round(float(rf), 4))
_, rt, rf = best
p = (tibia < rt) | (foot < rf)
print(f'Result: tibia radius {rt * 1000:.0f} mm, foot radius {rf * 1000:.0f} mm')
for label, mask in (('fit', fit), ('held out', ~fit)):
    print(f'  {label:9s}: {int((truth & ~p & mask).sum())} of {int((truth & mask).sum())} contacts missed, {int((~truth & p & mask).sum())} of {int((~truth & mask).sum())} false alarms')
print('poses must stay clear with these radii:')
model.body_collision.update(tibia_radius=rt, foot_radius=rf)
for pose in ('standby', 'laydown', 'neutral'):
    q = model.pose_q(pose)
    print(f"  {pose}: min gap {min(body_clearances(model, q).values()) * 1000:.1f} mm, limits {'ok' if not check_joint_limits(model, q) else 'violated'}")
