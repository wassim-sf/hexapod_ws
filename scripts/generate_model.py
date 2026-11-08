#!/usr/bin/env python3
"""Generate meshes/makeyourpet and config/robot_parameters.yaml from the MIT STL parts (assembly in parts_cad.py).

Masses and inertias are estimates: see docs/robot_parameters.md.
"""
import math
import pathlib
import sys

import numpy as np
import trimesh
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import parts_cad as cad  # noqa: E402

PKG = pathlib.Path(__file__).resolve().parents[1] / 'src/hexapod_description'
OUT = PKG / 'meshes/makeyourpet'

# ---- Print estimate from the brief ----
RHO_PLA, SHELL_MM, INFILL = 1.24e-3, 1.2, 0.10   # g/mm3; 3 walls of 0.4 mm; 10% infill
# ---- Parts. EST unless noted ----
M_SERVO = 55.0            # g, datasheet value (consistent across sources)
M_SCREW = 0.20            # g, M2 x 8 self-tapping, from nominal volume
SCREWS_COXA, SCREWS_FEMUR, SCREWS_TIBIA, SCREWS_BODY = 8, 8, 12, 60   # counted from the holes; the guide only says "pack of 100"
M_SWITCH, M_CAP, M_WIRES_LEG = 1.5, 0.5, 12.0
FLOOR = -11.5       # top of the bottom cover floor; boards sit 0.3 mm above it in the fit check
TRAY_TOP = 7.4      # printed Pi tray, just above the 16 mm tall floor boards
ELECTRONICS = [  # name, grams, size (x, y, z) mm, centre (x, y, z) mm in base_link. Layout checked by scripts/check_bays.py
    ('lipo_3s_2200', 185.0, (105, 34, 24), (0, 0, -26.3)),
    ('bec_6v_20a', 60.0, (50, 28, 20), (0, -32.0, -24.3)),
    ('pca9685_a', 10.0, (62.5, 25.4, 16), (-22.75, 14.2, FLOOR + 8.3)),
    ('pca9685_b', 10.0, (62.5, 25.4, 16), (-22.75, -14.2, FLOOR + 8.3)),
    ('esp32', 10.0, (51, 28, 13), (0, 32.0, -20.8)),
    ('buck_5v', 25.0, (45, 25, 15), (31.5, -14.2, FLOOR + 7.8)),
    ('pi_tray', 20.0, (92, 60, 2.4), (0, 0, TRAY_TOP - 1.2)),
    ('pi5', 46.0, (85, 56, 1.5), (0, 0, TRAY_TOP + 5.75)),
    ('active_cooler', 30.0, (63.5, 42.5, 13.7), (-4.0, 5.3, TRAY_TOP + 13.35)),
    ('imu', 3.0, (20.3, 15.6, 3.2), (0, 0, TRAY_TOP + 2.1)),
    ('servo_power_relay', 35.0, (28, 28, 25), (-52.0, 0, 41.0)),
    ('main_wiring_fuse', 50.0, (100, 60, 15), (0, 0, FLOOR + 8.3)),
]


def printed_grams(mesh):
    volume, area = abs(mesh.volume), mesh.area
    shell = min(volume, area * SHELL_MM)
    return RHO_PLA * (shell + INFILL * (volume - shell))


class Composite:
    """Rigid-body sum in grams and millimetres."""

    def __init__(self):
        self.items = []

    def add_mesh(self, mesh, grams):
        props = mesh.mass_properties
        self.items.append((grams, np.array(props['center_mass']), props['inertia'] * (grams / props['mass'])))

    def add_box(self, grams, extents, t):
        e = np.asarray(extents, float)
        i = grams / 12.0 * np.diag([e[1] ** 2 + e[2] ** 2, e[0] ** 2 + e[2] ** 2, e[0] ** 2 + e[1] ** 2])
        r = t[:3, :3]
        self.items.append((grams, t[:3, 3].copy(), r @ i @ r.T))

    def add_point(self, grams, at):
        self.items.append((grams, np.asarray(at, float), np.eye(3) * 1e-3))

    def total(self):
        mass = sum(g for g, _, _ in self.items)
        com = sum(g * c for g, c, _ in self.items) / mass
        inertia = np.zeros((3, 3))
        for g, c, i in self.items:
            d = c - com
            inertia += i + g * ((d @ d) * np.eye(3) - np.outer(d, d))
        return mass, com, inertia


def mirror_y(comp):
    out = Composite()
    s = np.diag([1.0, -1.0, 1.0])
    out.items = [(g, s @ c, s @ i @ s) for g, c, i in comp.items]
    return out


def link_entry(comp):
    grams, com, i = comp.total()
    eig = np.linalg.eigvalsh(i)
    assert eig.min() > 0 and eig[0] + eig[1] >= eig[2] - 1e-9, f'invalid inertia {eig}'
    k = 1e-9
    return {'mass': round(grams / 1000, 5), 'com': [round(float(v) / 1000, 5) for v in com],
            'inertia': {n: float(f'{i[a, b] * k:.4e}') for n, a, b in (('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2))}}


rows = []


def build(parts, label, servos=(), screws=0, extra=0.0, extra_at=None):
    comp = Composite()
    printed = sum(printed_grams(cad.placed(n, t)) for _, n, t in parts)
    hardware = screws * M_SCREW
    factor = (printed + hardware) / printed
    for name, stl, t in parts:
        mesh = cad.placed(stl, t)
        g = printed_grams(mesh)
        comp.add_mesh(mesh, g * factor)
        rows.append((label, name, abs(mesh.volume) / 1e3, mesh.area / 100, g))
    if hardware:
        rows.append((label, f'{screws} x M2x8 screw', 0, 0, hardware))
    for t in servos:
        centre = transform_point(t, (cad.SHAFT_OFFSET, 0, (cad.GEAR_TOP - cad.FLANGE_TO_BACK) / 2))
        tt = t.copy()
        tt[:3, 3] = centre
        comp.add_box(M_SERVO, (cad.SERVO_L, cad.SERVO_W, cad.GEAR_TOP + cad.FLANGE_TO_BACK), tt)
        rows.append((label, 'MG996R', 0, 0, M_SERVO))
    if extra:
        comp.add_point(extra, extra_at)
        rows.append((label, 'switch, cap, wiring', 0, 0, extra))
    return comp


def transform_point(t, p):
    return (t @ np.array([*p, 1.0]))[:3]


# Left-type links; the YAML stores right-side values (the xacro flips y for left legs).
coxa = mirror_y(build(cad.COXA_PARTS, 'coxa', [cad.COXA_SERVO_T], SCREWS_COXA))
femur = mirror_y(build(cad.FEMUR_PARTS, 'femur', [], SCREWS_FEMUR))
tibia = mirror_y(build(cad.TIBIA_PARTS, 'tibia', [cad.TIBIA_SERVO_T], SCREWS_TIBIA, M_SWITCH + M_WIRES_LEG, (0, 0, -60.0)))
foot = mirror_y(build(cad.FOOT_PARTS, 'foot', [], 0, M_CAP, (0, 0, 4.0)))
body_parts = list(cad.body_parts())
for _, _, _, yaw in cad.LEGS:
    body_parts.append(('idler', 'idler', cad.idler_transform(cad.body_servo_t(yaw))))
    body_parts.append(('ring', 'ring', cad.ring_transform(cad.body_servo_t(yaw))))
base = build(body_parts, 'base', [cad.body_servo_t(y) for *_, y in cad.LEGS], SCREWS_BODY)
for name, g, ext, c in ELECTRONICS:
    base.add_box(g, ext, cad.transform(np.eye(3), c))
    rows.append(('base', name, 0, 0, g))


# ---- Meshes ----
def small(mesh, faces):
    return mesh.simplify_quadric_decimation(face_count=faces) if len(mesh.faces) > faces else mesh


def write(mesh, name, faces=10 ** 7):
    m = small(mesh, faces).copy()
    m.apply_scale(0.001)
    m.export(OUT / f'{name}.stl')


OUT.mkdir(parents=True, exist_ok=True)
(OUT / 'LICENSE').write_text((cad.SOURCE / 'LICENSE').read_text())
(OUT / 'README.md').write_text('Derived from github.com/MakeYourPet/hexapod (MIT, see LICENSE), commit ' + cad.SOURCE_COMMIT + '.\n'
                               'Generated by scripts/generate_model.py. The servos and foot switches are our own placeholder boxes.\n')
write(trimesh.util.concatenate([small(cad.placed(n, t), 12000) for _, n, t in body_parts]), 'base_link')
servo_visual = cad.servo_mesh()
body_servos = []
for _, _, _, yaw in cad.LEGS:
    s = servo_visual.copy()
    s.apply_transform(cad.body_servo_t(yaw))
    body_servos.append(s)
write(trimesh.util.concatenate(body_servos), 'base_servos')
for name, parts, faces in (('coxa', cad.COXA_PARTS, 14000), ('femur', cad.FEMUR_PARTS, 14000), ('tibia', cad.TIBIA_PARTS, 12000), ('foot', cad.FOOT_PARTS, 2000)):
    mesh = trimesh.util.concatenate([small(cad.placed(n, t), faces) for _, n, t in parts])
    write(mesh, f'{name}_left')
    write(cad.mirror(mesh), f'{name}_right')
for name, t in (('coxa', cad.COXA_SERVO_T), ('tibia', cad.TIBIA_SERVO_T)):
    s = servo_visual.copy()
    s.apply_transform(t)
    if name == 'tibia':
        s = trimesh.util.concatenate([s, cad.switch_mesh()])  # the foot switch, in the same dark colour
    write(s, f'{name}_servo_left')
    write(cad.mirror(s), f'{name}_servo_right')


# ---- Poses ----
def ik(radial, up):
    f, t, c = cad.FEMUR, cad.TIBIA, cad.COXA
    a, b = radial - c, up
    e = -math.acos((a * a + b * b - f * f - t * t) / (2 * f * t))
    q3 = e + math.pi / 2
    q2 = math.atan2(b, a) - math.atan2(t * math.sin(e), f + t * math.cos(e))
    return [0.0, round(q2, 6), round(q3, 6)]


STANDBY = ik(120.0, -85.0)   # foot 120 mm out from the hip, 85 mm below: close to the project's MODE_STANDARD for this femur
LAYDOWN = ik(140.0, -cad.HIP_Z - 42.0)  # body resting on the battery bars

# ---- Joint limits (rad): servo travel windows intersected with mesh contact from sweep_limits.py ----
SERVO_CENTER = {'coxa': 0.0, 'femur': 35.2, 'tibia': -21.4}   # femur: hub screw pattern; tibia: knee hub pattern and the servo axis
LIMITS_DEG = {'coxa': (-55.0, 55.0), 'femur': (-54.0, 95.0), 'tibia': (-90.0, 68.0)}  # tibia stops at -90: contact is at -96, and below -90 the closed-form IK has two answers
TORQUE = round(11.0 * 9.80665 / 100.0, 4)       # 11 kgf.cm at 6 V
VELOCITY = round(float(np.radians(60.0)) / 0.14, 3)  # 0.14 s/60 deg at 6 V, no load
ALSO = {'axis': {'coxa': [0, 0, 1], 'femur': [0, -1, 0], 'tibia': [0, -1, 0]}}


def limit(name):
    lo, hi = LIMITS_DEG[name]
    return {'axis': ALSO['axis'][name], 'lower': round(math.radians(lo), 6), 'upper': round(math.radians(hi), 6), 'lower_deg': lo, 'upper_deg': hi,
            'effort': TORQUE, 'velocity': VELOCITY}


m = lambda v: round(float(v) * 0.001, 6)


def box_mm(xmin, xmax, ymin, ymax, zmin, zmax, yaw=0.0):
    return {'size': [m(xmax - xmin), m(ymax - ymin), m(zmax - zmin)], 'xyz': [m((xmin + xmax) / 2), m((ymin + ymax) / 2), m((zmin + zmax) / 2)], 'yaw': round(yaw, 6)}


def hip_box(yaw, r0, r1, half_w, z0, z1):
    """A box in the leg's radial frame at its hip, expressed in base_link."""
    a = math.radians(yaw)
    xy = cad.hip_xy(yaw)
    c = (r0 + r1) / 2
    return {'size': [m(r1 - r0), m(2 * half_w), m(z1 - z0)], 'xyz': [m(xy[0] + c * math.cos(a)), m(xy[1] + c * math.sin(a)), m((z0 + z1) / 2)], 'yaw': round(a, 6)}


base_boxes = [box_mm(-58.3, 58.3, -38.3, 38.3, 0.0, 16.0), box_mm(-62.0, 68.0, -48.0, 48.0, -14.0, 0.0), box_mm(-70.0, 70.0, -50.0, 50.0, 16.0, 28.2), box_mm(-52.5, 52.5, -24.0, 24.0, -42.0, -14.0)]
for _, _, _, yaw in cad.LEGS:
    base_boxes.append(hip_box(yaw, -30.5, 10.1, 10.0, -8.0, 30.0))  # coxa servo body
legs = []
for leg_id, name, side, yaw in cad.LEGS:
    xy = cad.hip_xy(yaw)
    legs.append({'id': leg_id, 'name': name, 'side': side, 'mirror': 1 if side == 'right' else -1, 'yaw': round(math.radians(yaw), 6), 'yaw_deg': yaw, 'x': m(xy[0]), 'y': m(xy[1])})
zf = cad.COXA_Z_LOCAL
coxa_boxes = [box_mm(-12.5, 53.0, -8.0, 12.5, -20.25, -14.25), box_mm(-6.0, 53.0, -8.0, 8.0, 33.75, 39.75), box_mm(24.0, 31.0, -8.0, 8.0, -14.25, 33.75), box_mm(33.0, 53.0, -22.0, 8.0, -10.1, 30.6)]
femur_boxes = [box_mm(-12.5, 72.5, 19.0, 24.0, -12.5, 25.0), box_mm(-9.5, 67.5, -36.0, -29.0, -7.0, 25.0), box_mm(41.0, 55.0, -29.0, 19.0, 23.0, 29.0)]
tibia_boxes = [box_mm(-10.1, 30.6, -9.0, 9.0, -14.0, -2.0)]
# Tibia boxes follow the bar and servo in the rotated tibia frame, so the bar is approximated by the leg line.
tibia_boxes = [box_mm(-14.0, 14.0, -10.0, 10.0, -30.0, 12.0), box_mm(4.0, 30.0, -20.0, 20.0, -108.0, 16.0), box_mm(-2.0, 14.0, -4.5, 4.5, -98.0, -30.0), box_mm(-1.0, 28.0, -8.5, 8.5, -125.0, -98.0)]
sections = [
    ('MakeYourPet hexapod with the Robot Lk part selection. SI units, radians.',
     {'robot': {'name': 'hexapod', 'variant': 'makeyourpet_chipo_femur_long_tibia', 'source': 'github.com/MakeYourPet/hexapod 7d8fc80 (MIT)'}}),
    ('Visual meshes generated by scripts/generate_model.py from the MIT parts. Servos are placeholder boxes.',
     {'visual': {'mesh_subdir': 'makeyourpet', 'servo_model': 'placeholder_from_datasheet'}}),
    ('MEASURED from STL. base_link: x forward, y left, z up, origin at the body centre on the frame underside.',
     {'body': {'hip_z': m(cad.HIP_Z), 'frame_thickness': m(cad.FRAME_THICKNESS), 'across_x': 0.2054, 'across_y': 0.198, 'body_bottom_z': -0.042, 'top_cover_z': 0.0282}}),
    ('MEASURED from STL. Published config: COXA_LEN 43; Chipo FEMUR_LEN 60; main TIBIA_LEN 134.',
     {'leg_geometry': {'coxa_length': m(cad.COXA), 'femur_length': m(cad.FEMUR), 'tibia_length': m(cad.TIBIA)}}),
    ('Leg order as in earlier steps: right front to rear, then left. mirror: right +1, left -1.', {'legs': legs}),
    ('Axes are in the leg frame (x outward, z up), the same on all six legs. effort and velocity: MG996R at 6.0 V.',
     {'joints': {n: limit(n) for n in ('coxa', 'femur', 'tibia')}}),
    ('Named poses in radians [coxa, femur, tibia].', {'poses': {'neutral': [0.0, 0.0, 0.0], 'standby': STANDBY, 'laydown': LAYDOWN}}),
    ('Joint angle (deg) when the servo is at its centre pulse. For the hardware interface.',
     {'servo_center_deg': SERVO_CENTER}),
    ('Sources and conflicts: docs/robot_parameters.md.',
     {'servo': {'model': 'MG996R', 'mass': m(M_SERVO), 'stall_torque_4v8': round(9.4 * 9.80665 / 100, 4), 'stall_torque_6v': TORQUE,
                'speed_4v8': round(float(np.radians(60)) / 0.17, 3), 'speed_6v': VELOCITY, 'stall_current_6v': 2.5, 'running_current_6v': [0.5, 0.9],
                'travel_deg': 180}}),
    ('ESTIMATED. Right-side leg values in the link frame; the xacro flips y terms for left legs. kg, m, kg.m2.',
     {'links': {'base': link_entry(base), 'coxa': link_entry(coxa), 'femur': link_entry(femur), 'tibia': link_entry(tibia), 'foot': link_entry(foot)}}),
    ('Simplified boxes fitted to the measured parts. Right-side values; foot is a cylinder.',
     {'collision': {'base': base_boxes, 'coxa': [mirror_box for mirror_box in coxa_boxes], 'femur': femur_boxes, 'tibia': tibia_boxes,
                    'foot': {'radius': 0.0045, 'length': 0.009, 'z': 0.0045}}}),
    ('DERIVED. Capsules for the neighbouring-leg check, fitted by scripts/calibrate_leg_collision.py.',
     {'leg_collision': {'femur_radius': 0.032, 'tibia_upper_radius': 0.020, 'tibia_lower_radius': 0.004, 'tibia_split': 0.095}}),
    ('Foot contact: one microswitch per foot, pressed when the rod rises rod_travel into the ms-holder. esp32_gpio is by leg id.\n'
     '# ESTIMATED for simulation, in N and s: contact counts above sim_force_threshold, clears below sim_release_force, and reports after sim_delay.',
     {'foot_contact': {'rod_travel': m(cad.ROD_TRAVEL), 'update_rate_hz': 100, 'topic': 'foot_contact/leg_{id}', 'esp32_gpio': {1: 18, 2: 19, 3: 23, 4: 25, 5: 26, 6: 27},
                       'wiring': 'C to GPIO with INPUT_PULLUP, NO to GND, NC unused; pressed reads LOW',
                       'sim_force_threshold': 1.5, 'sim_release_force': 0.5, 'sim_delay': 0.01}}),
    ('Other ESP32 pins. Servo power is off unless servo_power_gpio is driven HIGH; PCA9685 OE is active low, with an external pull-up keeping outputs off at boot.',
     {'electronics': {'servo_power_gpio': 32, 'pca9685_oe_gpio': 33, 'i2c_gpio': {'sda': 21, 'scl': 22}, 'uart_gpio': {'tx': 17, 'rx': 16}}}),
    ('PLACEHOLDER position: on the Pi tray under the Pi, at the body centre. BNO055 or MPU6050, not yet chosen.',
     {'imu': {'xyz': [0.0, 0.0, 0.0095], 'status': 'placeholder'}}),
]


def plain(o):
    if isinstance(o, dict):
        return {k: plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [plain(v) for v in o]
    return o.item() if isinstance(o, np.generic) else o


text = ['# Single source of truth for the hexapod model: hexapod_description, the kinematics library and the RL environment.',
        '# Generated by scripts/generate_model.py. Provenance of every number: docs/robot_parameters.md.', '']
for comment, data in sections:
    text.append(f'# {comment}')
    text.append(yaml.safe_dump(plain(data), sort_keys=False, default_flow_style=None, width=120).rstrip())
    text.append('')
(PKG / 'config/robot_parameters.yaml').write_text('\n'.join(text))

if '-v' in sys.argv:
    print(f"{'link':6s} {'part':24s} {'V cm3':>7s} {'A cm2':>7s} {'g':>7s}")
    for label, name, v, a, g in rows:
        print(f'{label:6s} {name:24s} {v:7.2f} {a:7.2f} {g:7.2f}')
leg_total = 0.0
for name, comp in (('base', base), ('coxa', coxa), ('femur', femur), ('tibia', tibia), ('foot', foot)):
    g, c, _ = comp.total()
    print(f'TOTAL {name:6s} {g:8.1f} g  com mm {np.round(c, 1)}')
    if name != 'base':
        leg_total += g
print(f'one leg {leg_total:.1f} g, robot {base.total()[0] + 6 * leg_total:.1f} g, standby {np.degrees(STANDBY).round(1)}, laydown {np.degrees(LAYDOWN).round(1)}')
