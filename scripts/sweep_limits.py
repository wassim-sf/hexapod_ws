#!/usr/bin/env python3
"""Find mechanical joint limits by mesh collision: printed parts plus MG996R body outlines.

Horns, idlers, rings, screws and wires are left out, because they touch by design or are not modelled.
"""
import pathlib
import sys

import numpy as np
import trimesh

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import parts_cad as cad  # noqa: E402
from hexapod_kinematics import RobotModel, link_frames  # noqa: E402

model = RobotModel.load()


def servo_body(t):
    s = cad.servo_mesh(body_only=True)
    s.apply_transform(t)
    return s


def scaled(meshes, side='left'):
    mesh = trimesh.util.concatenate(meshes)
    mesh.apply_scale(0.001)
    return cad.mirror(mesh) if side == 'right' else mesh


def leg_sets(side):
    return {
        'coxa': scaled([cad.placed('Left-coxa-996', cad.COXA_T), servo_body(cad.COXA_SERVO_T)], side),
        'femur': scaled([cad.placed('Left-femur-996', cad.FEMUR_T)], side),
        'tibia': scaled([cad.placed(n, t) for n, t in (('Left-tibia', cad.TIBIA_T), ('shield', cad.SHIELD_T), ('ms-holder', cad.FOOT_T), ('rod', cad.FOOT_T))]
                        + [servo_body(cad.TIBIA_SERVO_T)], side),
        'foot': scaled([cad.placed('tip', cad.TIP_T)], side),
    }


SETS = {'left': leg_sets('left'), 'right': leg_sets('right')}
body = [cad.placed(n, t) for _, n, t in cad.body_parts()] + [servo_body(cad.body_servo_t(y)) for *_, y in cad.LEGS]
BODY = trimesh.collision.CollisionManager()
BODY.add_object('body', scaled(body))
ORDER = {'coxa': 0, 'femur': 1, 'tibia': 2, 'foot': 3}


class Leg:
    def __init__(self, leg, tag=''):
        self.leg = leg
        self.manager = trimesh.collision.CollisionManager()
        for name, mesh in SETS[leg.side].items():
            self.manager.add_object(tag + name, mesh)
        self.tag = tag

    def pose(self, q):
        frames = link_frames(model, self.leg.index, q)
        for name in SETS[self.leg.side]:
            self.manager.set_transform(self.tag + name, frames[name])

    def contacts(self):
        """Contact with the body, or between non-adjacent or adjacent links."""
        hit_b, n_b = self.manager.in_collision_other(BODY, return_names=True)
        hit_s, n_s = self.manager.in_collision_internal(return_names=True)
        pairs = {tuple(sorted(p)) for p in n_b} | {tuple(sorted(p)) for p in n_s}
        return sorted(pairs - {('foot', 'tibia')})  # the tip sits on the rod by design


def first_contact(leg, joint, sign, base_q, maxdeg=135):
    q = np.radians(base_q).astype(float)
    start = np.degrees(q[joint])
    obj = Leg(leg)
    obj.pose(q)
    if obj.contacts():
        return None, obj.contacts()
    for d in range(1, maxdeg + 1):
        q[joint] = np.radians(start + sign * d)
        obj.pose(q)
        c = obj.contacts()
        if c:
            return start + sign * (d - 1), c
    return None, []


def toward(i, j, angle):
    wrap = (model.legs[j].yaw - model.legs[i].yaw + np.pi) % (2 * np.pi) - np.pi
    s = np.sign(wrap)
    return s * angle, -s * angle


if __name__ == '__main__':
    stand = np.degrees(model.poses['standby'])
    for leg in (model.legs[1], model.legs[4], model.legs[0], model.legs[3]):
        print(f'\n--- leg_{leg.id} {leg.name}, others at standby {stand.round(1)}')
        for j, name in enumerate(('coxa', 'femur', 'tibia')):
            for s in (1, -1):
                a, c = first_contact(leg, j, s, stand)
                print(f'{name:6s} {"+" if s > 0 else "-"}: last clear {a}  {c}')
    print('\n--- tibia limits at other femur angles (leg_2)')
    for f in (-55, -30, 0, 30, 60, 90, 110):
        out = []
        for s in (1, -1):
            a, c = first_contact(model.legs[1], 2, s, np.array([0.0, f, -20.0]))
            out.append(f'{"+" if s > 0 else "-"}: {a} {c[:1]}')
        print(f'femur {f:4d}: ' + ' | '.join(out))
    print('\n--- femur limits at other tibia angles (leg_2)')
    for t in (-100, -60, -30, 0, 40):
        out = []
        for s in (1, -1):
            a, c = first_contact(model.legs[1], 1, s, np.array([0.0, 40.0, t]))
            out.append(f'{"+" if s > 0 else "-"}: {a} {c[:1]}')
        print(f'tibia {t:4d}: ' + ' | '.join(out))
    print('\n--- neighbouring legs swinging toward each other')
    for pair in model.neighbour_pairs:
        i, j = pair
        a, b = Leg(model.legs[i], 'a_'), Leg(model.legs[j], 'b_')
        for pose in (stand, (0, 0, 0), (0, 60, -20)):
            found = None
            for d in range(0, 91):
                x, y = toward(i, j, np.radians(d))
                a.pose([x, np.radians(pose[1]), np.radians(pose[2])])
                b.pose([y, np.radians(pose[1]), np.radians(pose[2])])
                hit, names = a.manager.in_collision_other(b.manager, return_names=True)
                if hit:
                    found = (d, sorted({tuple(sorted(n)) for n in names})[:2])
                    break
            print(f'legs {model.legs[i].id},{model.legs[j].id} pose {np.round(pose, 0)}: first contact {found}')
