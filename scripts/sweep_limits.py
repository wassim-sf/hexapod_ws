#!/usr/bin/env python3
"""Find mechanical joint limits by sweeping meshes for collisions (printed parts + MG92B body boxes).

Parts that sit flush by design (pin-side plates) get a 0.3 mm test gap so that sliding contact
does not count as a collision. Horns, screws and wires are not modelled.
"""
import sys, pathlib
import numpy as np, trimesh
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import hexapod_cad as cad
GAP = 0.3
def shifted(parts, shifts):
    out = []
    for name, rel, m in cad.placed(parts):
        if name in shifts:
            m.apply_translation(shifts[name])
        out.append((name, m))
    return out
def box(ext, c):
    b = trimesh.creation.box(extents=ext); b.apply_translation(c); return b
# MG92B body (without shaft): 22.8 x 12 x ~25 mm. Case height along the shaft is estimated.
def femur_set():
    s = shifted(cad.FEMUR_PARTS, {})
    for i, x in enumerate((5.3, 38.3)):
        s.append((f'servo{i}', box((22.8, 25, 12), (x, 3.7, 0))))
    return s
COXA_SET = shifted(cad.COXA_PARTS, {'joint_bottom_hip': (0, 0, -GAP), 'joint_bottom_femur': (0, -GAP, 0)})
FEMUR_SET = femur_set()
TIBIA_SET = shifted(cad.TIBIA_PARTS, {'foot_bottom': (0, -GAP, 0)}) + [
    (n, m.apply_translation((0, 0, -cad.TIBIA))) for n, m in shifted(cad.FOOT_PARTS, {})]
BODY_SET = [('body', trimesh.util.concatenate([m for _, _, m in cad.placed(cad.body_parts())]))]
for k in range(6):  # coxa servo bodies live in the body
    a = np.radians(cad.LEGS[k][3]); r = cad.HIP_RADIUS - 5.3
    b = box((22.8, 12, 25), (r, 0, 24.5)); b.apply_transform(cad.t_rz(cad.LEGS[k][3])); BODY_SET.append((f'coxa_servo{k}', b))
def world(side, yaw, q):
    tc, tf_, tt, tfoot = cad.leg_transforms(yaw, q)
    out = {}
    for label, parts, t in (('coxa', COXA_SET, tc), ('femur', FEMUR_SET, tf_), ('tibia', TIBIA_SET, tt)):
        ms = []
        for n, m in parts:
            mm = cad.mirror(m) if side == 'left' else m.copy()
            mm.apply_transform(t); ms.append(mm)
        out[label] = trimesh.util.concatenate(ms)
    return out
BODY_CM = trimesh.collision.CollisionManager()
BODY_CM.add_object('body', trimesh.util.concatenate([m for _, m in BODY_SET]))
def collide(leg_meshes, others=None):
    """Leg links against the body and each other. Body-internal contact is ignored."""
    cm = trimesh.collision.CollisionManager()
    for k, v in leg_meshes.items():
        cm.add_object(k, v)
    if others:
        for k, v in others.items():
            cm.add_object('o_' + k, v)
    hit, names = cm.in_collision_internal(return_names=True)
    hit2, names2 = cm.in_collision_other(BODY_CM, return_names=True)
    return hit or hit2, set(names) | set(names2)
def sweep(joint_i, sign, side='right', yaw=-90.0, base=(0, 0, 0), maxdeg=120):
    for d in range(1, maxdeg + 1):
        q = list(base); q[joint_i] = sign * d
        hit, names = collide(world(side, yaw, q))
        if hit:
            return d - 1, sorted(names)
    return None, []
if __name__ == '__main__':
    hit0, names0 = collide(world('right', -90.0, (0, 0, 0)))
    print('neutral self-collision:', hit0, sorted(names0))
    for side, yaw in (('right', -90.0), ('left', 90.0)):
        print(f'--- {side}, yaw {yaw}')
        for i, jn in enumerate(('coxa', 'femur', 'tibia')):
            for s in (1, -1):
                d, names = sweep(i, s, side, yaw)
                print(f'{jn:6s} {"+" if s > 0 else "-"}: first contact after {d} deg  {names}')
    print('--- tibia with the femur at standby and at extremes')
    for f in (-75, -30, 30, 65, 75):
        for s in (1, -1):
            d, names = sweep(2, s, 'right', -90.0, (0, f, 0))
            print(f'femur {f:4d}: tibia {"+" if s > 0 else "-"}: {d} deg {names}')
    print('--- neighbouring legs swinging toward each other (leg1 yaw -30 with q, leg2 yaw -90 with -q)')
    for f, t in ((0, 0), (30, -15), (65, 0)):
        for d in range(1, 91):
            a = world('right', -30.0, (-d, f, t)); b = world('right', -90.0, (d, f, t))
            hit, names = collide(a, b)
            if hit:
                print(f'pose f={f},t={t}: contact after {d-1} deg each {sorted(names)}'); break
        else:
            print(f'pose f={f},t={t}: no contact up to 90 deg')
