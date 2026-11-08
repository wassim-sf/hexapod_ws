#!/usr/bin/env python3
"""Generate hexapod_description/meshes and config/robot_parameters.yaml from the STL parts.

Masses and inertias are estimates (see docs/robot_parameters.md). Geometry comes from hexapod_cad.py.
"""
import pathlib, sys
import numpy as np, trimesh, yaml
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import hexapod_cad as cad

PKG = pathlib.Path(__file__).resolve().parents[1] / 'src/hexapod_description'

# ---- Estimates: material, print and hardware ----
RHO_PLA = 1.24e-3         # g/mm3, typical PLA
RHO_TPU = 1.21e-3         # g/mm3, typical TPU (the README recommends TPU for the foot tips)
SHELL = 0.8               # mm, walls/top/bottom, 2 perimeters of a 0.4 mm nozzle
INFILL = 0.20             # the brief's infill
M_SCREW12, M_NUT, M_SCREW6 = 0.35, 0.12, 0.20   # g, steel, from nominal volume
M_PIN = 0.59              # g, M4 x 6 steel pin, from volume
M_BEARING = 0.50          # g, MR74-2RS
M_SERVO = 13.8            # g, datasheet
SERVO_EXT = np.array([22.8, 31.0, 12.0])   # mm, length along x, shaft axis, width. Datasheet.
M_CELL, M_BOARD, M_SWITCH, M_WIRING = 46.0, 30.0, 10.0, 15.0   # g
# Hardware counts per link. The README only gives totals (180 M2x12, 216 nuts, 36 M2x6, 18 pins, 18 bearings).
SCREWS_PER_LINK = dict(coxa=8, femur=12, tibia=7)
BASE_SCREWS = 180 - 6 * sum(SCREWS_PER_LINK.values())
PINS = dict(coxa=2, tibia=1)
BEARINGS = dict(femur=2)
BASE_BEARINGS = 6


def printed_mass(m, rho=RHO_PLA):
    volume, area = abs(m.volume), m.area
    shell = min(volume, area * SHELL)
    return rho * (shell + INFILL * (volume - shell))


class Composite:
    """Rigid-body sum. Internally: grams, millimetres, frame = link frame."""
    def __init__(self):
        self.items = []

    def add_mesh(self, m, grams):
        props = m.mass_properties
        inertia = props['inertia'] * (grams / props['mass'])
        self.items.append((grams, np.array(props['center_mass']), inertia))
        return grams

    def add_box(self, grams, ext, center):
        e = np.asarray(ext, float)
        i = grams / 12.0 * np.diag([e[1]**2 + e[2]**2, e[0]**2 + e[2]**2, e[0]**2 + e[1]**2])
        self.items.append((grams, np.asarray(center, float), i))

    def total(self):
        mass = sum(g for g, _, _ in self.items)
        com = sum(g * c for g, c, _ in self.items) / mass
        inertia = np.zeros((3, 3))
        for g, c, i in self.items:
            d = c - com
            inertia += i + g * ((d @ d) * np.eye(3) - np.outer(d, d))
        return mass, com, inertia


def link_entry(comp):
    grams, com, i = comp.total()
    eig = np.linalg.eigvalsh(i)
    assert eig.min() > 0 and eig[0] + eig[1] >= eig[2] - 1e-9, f'invalid inertia {eig}'
    k = 1e-9  # g mm2 -> kg m2
    return {
        'mass': round(grams / 1000.0, 5),
        'com': [round(float(v) / 1000.0, 5) for v in com],
        'inertia': {name: float(f'{i[a, b] * k:.4e}') for name, a, b in
                    (('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2))},
    }


rows = []


def build(parts, label, screws=0, pins=0, bearings=0, servos=(), rho=RHO_PLA, servo_screws=None):
    comp = Composite()
    placed = cad.placed(parts)
    printed = sum(printed_mass(m, rho) for _, _, m in placed)
    n_servo_screws = len(servos) * 2 if servo_screws is None else servo_screws
    hardware = screws * (M_SCREW12 + M_NUT) + pins * M_PIN + bearings * M_BEARING + n_servo_screws * M_SCREW6
    factor = (printed + hardware) / printed
    for name, rel, m in placed:
        g = printed_mass(m, rho)
        comp.add_mesh(m, g * factor)
        rows.append((label, name, abs(m.volume) / 1e3, m.area / 1e2, g))
    for center in servos:
        comp.add_box(M_SERVO, SERVO_EXT, center)
    if hardware:
        rows.append((label, f'hardware ({screws} screw+nut, {pins} pin, {bearings} bearing, {n_servo_screws} M2x6)', 0, 0, hardware))
    if servos:
        rows.append((label, f'{len(servos)} x MG92B', 0, 0, len(servos) * M_SERVO))
    return comp


# Shaft top sits just below the outer plate face: y = 24.16 - 2 = 22.2 mm in the link frame.
SERVO_Y = 22.2 - SERVO_EXT[1] / 2
femur = build(cad.FEMUR_PARTS, 'femur', SCREWS_PER_LINK['femur'], bearings=BEARINGS['femur'],
              servos=[(5.3, SERVO_Y, 0), (38.3, SERVO_Y, 0)])
coxa = build(cad.COXA_PARTS, 'coxa', SCREWS_PER_LINK['coxa'], pins=PINS['coxa'])
tibia = build(cad.TIBIA_PARTS, 'tibia', SCREWS_PER_LINK['tibia'], pins=PINS['tibia'])
foot = build(cad.FOOT_PARTS, 'foot', rho=RHO_TPU)

# Base: coxa servo shafts share the femur-link geometry; z_top = hip_z + 22.2
base = build(cad.body_parts(), 'base', BASE_SCREWS, bearings=BASE_BEARINGS, servo_screws=12)
coxa_servo_z = cad.HIP_Z + 22.2 - SERVO_EXT[1] / 2
for _, _, _, yaw in cad.LEGS:
    a = np.radians(yaw)
    r = cad.HIP_RADIUS - 5.3
    t = cad.rz(yaw)
    i = M_SERVO / 12.0 * np.diag([SERVO_EXT[2]**2 + SERVO_EXT[1]**2, SERVO_EXT[0]**2 + SERVO_EXT[1]**2, SERVO_EXT[0]**2 + SERVO_EXT[2]**2])
    base.items.append((M_SERVO, np.array([r * np.cos(a), r * np.sin(a), coxa_servo_z]), t @ i @ t.T))
rows.append(('base', '6 x MG92B', 0, 0, 6 * M_SERVO))
# Non-printed parts. Positions are estimates from the photos: cells and switch on the battery plate, board in the hub.
base.add_box(2 * M_CELL, (65, 37, 18), (0, 0, cad.ARM_HEIGHT + cad.BATTERY_PLATE + 9))
base.add_box(M_BOARD, (60, 50, 12), (0, 0, 14))
base.add_box(M_SWITCH, (15, 15, 25), (0, 30, cad.ARM_HEIGHT + cad.BATTERY_PLATE + 12))
base.add_box(M_WIRING, (100, 100, 15), (0, 0, 14))
for name, g in (('2 x 18650 cell', 2 * M_CELL), ('controller board', M_BOARD), ('toggle switch', M_SWITCH), ('wiring (controller side)', M_WIRING)):
    rows.append(('base', name, 0, 0, g))


# ---- Meshes (visual only, metres, decimated) ----
def write_mesh(mesh, name, faces):
    m = mesh.simplify_quadric_decimation(face_count=faces) if len(mesh.faces) > faces else mesh.copy()
    m.apply_scale(0.001)
    m.export(PKG / 'meshes' / f'{name}.stl')
    return len(m.faces)


(PKG / 'meshes').mkdir(exist_ok=True)
# Decimate the body part by part so the large flat base plate cannot use up the dome's face budget.
BODY_FACES = {'body_base': 20000, 'body_top': 8000, 'body_battery': 6000, 'body_head': 12000, 'body_side': 1500, 'body_servo_side': 1500}


def body_visual():
    out = []
    for name, _, m in cad.placed(cad.body_parts()):
        limit = BODY_FACES[next(k for k in sorted(BODY_FACES, key=len, reverse=True) if name.startswith(k))]
        out.append(m.simplify_quadric_decimation(face_count=limit) if len(m.faces) > limit else m)
    return trimesh.util.concatenate(out)


counts = {'base_link': write_mesh(body_visual(), 'base_link', 10 ** 7)}
for name, parts in (('coxa', cad.COXA_PARTS), ('femur', cad.FEMUR_PARTS), ('tibia', cad.TIBIA_PARTS), ('foot', cad.FOOT_PARTS)):
    merged = cad.merged(parts)  # leg parts stay at full STL resolution
    counts[f'{name}_right'] = write_mesh(merged, f'{name}_right', 10 ** 7)
    counts[f'{name}_left'] = write_mesh(cad.mirror(merged), f'{name}_left', 10 ** 7)
print('mesh faces', counts)

# ---- YAML ----
C = cad.CROSS_HALF


def mm_to_m(v):
    return round(float(v) * 0.001, 6)


mm = 0.001


def box_spec(xmin, xmax, ymin, ymax, zmin, zmax):
    return {'size': [round((xmax - xmin) * mm, 5), round((ymax - ymin) * mm, 5), round((zmax - zmin) * mm, 5)],
            'xyz': [round((xmin + xmax) / 2 * mm, 5), round((ymin + ymax) / 2 * mm, 5), round((zmin + zmax) / 2 * mm, 5)]}


legs = []
for leg_id, name, side, yaw in cad.LEGS:
    y = np.radians(yaw)
    legs.append({'id': leg_id, 'name': name, 'side': side, 'mirror': 1 if side == 'right' else -1,
                 'yaw': round(float(y), 6), 'yaw_deg': yaw,
                 'x': round(float(cad.HIP_RADIUS * np.cos(y)) * mm, 6), 'y': round(float(cad.HIP_RADIUS * np.sin(y)) * mm, 6)})

EFFORT = round(3.5 * 9.80665 / 100.0, 4)       # 3.5 kgf.cm at 6.0 V, datasheet
VELOCITY = round(float(np.radians(60.0)) / 0.08, 3)   # 0.08 s/60 deg at 6.0 V, no load, datasheet
limit = lambda lo, hi: {'lower': round(float(np.radians(lo)), 6), 'upper': round(float(np.radians(hi)), 6),
                        'lower_deg': lo, 'upper_deg': hi, 'effort': EFFORT, 'velocity': VELOCITY}
sections = [
    ('Identity. RookiDroid "Mochi" variant: the MG92B build in the repo. SI units, radians.',
     {'robot': {'name': 'hexapod', 'variant': 'rookidroid_mochi', 'source_commit': '625d150e0d0315bf6a364b36b8aa8332ec558935'}}),
    ('MEASURED from STL. base_link: x forward, y left, z up, origin under the body centre at the base underside.',
     {'body': {'hip_radius': mm_to_m(cad.HIP_RADIUS), 'hip_z': mm_to_m(cad.HIP_Z), 'arm_height': mm_to_m(cad.ARM_HEIGHT),
               'across_arm_tips': 0.1836, 'across_flats': 0.16726, 'body_height': round(cad.BODY_HEIGHT * mm, 5)}}),
    ('MEASURED from STL. coxa/femur match mochi.json; tibia differs by 1.76 mm (mochi.json 0.08522).',
     {'leg_geometry': {'coxa_length': mm_to_m(cad.COXA), 'femur_length': mm_to_m(cad.FEMUR), 'tibia_length': mm_to_m(cad.TIBIA),
                       'foot_lateral_offset_not_modelled': round(cad.TIP_Y * mm, 5)}}),
    ('Leg order follows mochi.json. mirror: right +1, left -1.', {'legs': legs}),
    ('Axes are in the leg frame (x outward, z up), the same for all six legs. Limits: see docs/robot_parameters.md.\n'
     '# effort and velocity: MG92B datasheet at 6.0 V (stall torque, no-load speed).',
     {'joints': {'coxa': {'axis': [0, 0, 1], **limit(-45, 45)},
                 'femur': {'axis': [0, -1, 0], **limit(-75, 75)},
                 'tibia': {'axis': [0, -1, 0], **limit(-50, 75)}}}),
    ('Named poses in radians [coxa, femur, tibia]. standby and laydown are converted from mochi.json postures.',
     {'poses': {'neutral': [0.0, 0.0, 0.0],
                'standby': [0.0, round(float(np.radians(30)), 6), round(float(np.radians(-15)), 6)],
                'laydown': [0.0, round(float(np.radians(65)), 6), 0.0]}}),
    ('Datasheet values. Sources and conflicts: docs/robot_parameters.md.',
     {'servo': {'model': 'MG92B', 'mass': M_SERVO * mm, 'stall_torque_5v': round(3.1 * 9.80665 / 100, 4),
                'stall_torque_6v': EFFORT, 'speed_5v': round(float(np.radians(60) / 0.13), 3), 'speed_6v': VELOCITY,
                'travel_deg_500_2400us': 160, 'travel_deg_700_2300us': 150}}),
    ('ESTIMATED. Right-side leg values in the link frame; the xacro flips y terms for left legs. kg, m, kg.m2.',
     {'links': {'base': link_entry(base), 'coxa': link_entry(coxa), 'femur': link_entry(femur),
                'tibia': link_entry(tibia), 'foot': link_entry(foot)}}),
    ('Simplified shapes fitted to the measured parts (metres). Right-side values.',
     {'collision': {
         'base': {'hub': {'radius': 0.064, 'length': mm_to_m(cad.ARM_HEIGHT), 'z': mm_to_m(cad.ARM_HEIGHT / 2)},
                  'dome': {'radius': 0.0697, 'length': round((cad.BODY_HEIGHT - cad.ARM_HEIGHT) * mm, 5),
                           'z': round((cad.BODY_HEIGHT + cad.ARM_HEIGHT) / 2 * mm, 5)},
                  'arm': {'size': [0.0298, 0.0224, mm_to_m(cad.ARM_HEIGHT)], 'radial_center': 0.0769, 'z': mm_to_m(cad.ARM_HEIGHT / 2)}},
         'coxa': {'plate_top': box_spec(-12.5, 23, -7.5, 7.5, C, C + 3.4),
                  'plate_bottom': box_spec(-7.95, 23, -7.5, 7.5, -C - 3.6, -C),
                  'plate_horn_side': box_spec(13, 48.5, C, C + 3.4, -7.5, 7.5),
                  'plate_pin_side': box_spec(13, 43.95, -C - 3.6, -C, -7.5, 7.5),
                  'cross': {'x': mm_to_m(cad.COXA / 2), 'radius': round(C * mm, 5), 'length': 0.004}},
         'femur': {'block': box_spec(-10.5, 54.1, cad.BLOCK_LOW, cad.BLOCK_HIGH, -11.2, 11.2)},
         'tibia': {'plate_horn_side': box_spec(-7.5, 7.5, cad.BLOCK_LOW + cad.FOOT_WIDTH, cad.BLOCK_LOW + cad.FOOT_WIDTH + 3.4, -60, 14.5),
                   'plate_pin_side': box_spec(-8, 8, cad.BLOCK_LOW - 3.6, cad.BLOCK_LOW, -60, 8),
                   'ground_upper': box_spec(-2, 2, cad.TIP_Y - 19.75, cad.TIP_Y + 19.75, -60, -20),
                   'ground_lower': box_spec(-2, 2, cad.TIP_Y - 8, cad.TIP_Y + 8, -76.46, -60)},
         'foot': {'radius': 0.006, 'length': 0.009, 'z': 0.006}}}),
    ('PLACEHOLDER: the Mochi parts list has no IMU. Position assumed at the body centre, mid-hub.',
     {'imu': {'xyz': [0.0, 0.0, round(cad.ARM_HEIGHT / 2 * mm, 5)], 'status': 'placeholder'}}),
]


def plain(o):
    """Convert numpy scalars to Python values so YAML stays readable by any consumer."""
    if isinstance(o, dict):
        return {k: plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [plain(v) for v in o]
    return o.item() if isinstance(o, np.generic) else o


text = ['# Single source of truth for the hexapod model: hexapod_description, the kinematics library and the RL environment.',
        '# Generated by scripts/generate_model.py. Provenance of every number: docs/robot_parameters.md.', '']
for comment, data in sections:
    text.append(f'# {comment}')
    text.append(yaml.safe_dump(plain(data), sort_keys=False, default_flow_style=None, width=110).rstrip())
    text.append('')
(PKG / 'config').mkdir(exist_ok=True)
(PKG / 'config/robot_parameters.yaml').write_text('\n'.join(text))

# ---- Report ----
print(f"\n{'link':6s} {'part':58s} {'V cm3':>7s} {'A cm2':>7s} {'g':>7s}")
seen = {}
for label, name, v, a, g in rows:
    print(f'{label:6s} {name:58s} {v:7.2f} {a:7.2f} {g:7.2f}')
leg_total = 0
for n, comp in (('base', base), ('coxa', coxa), ('femur', femur), ('tibia', tibia), ('foot', foot)):
    g, c, _ = comp.total()
    print(f'TOTAL {n:6s} {g:8.1f} g  com mm {np.round(c, 1)}')
    if n != 'base':
        leg_total += g
total = base.total()[0] + 6 * leg_total
print(f'one leg {leg_total:.1f} g, robot {total:.1f} g, base servo count 6, femur servo count 12')
g = 9.80665
tip_reach = cad.COXA + cad.FEMUR * np.cos(np.radians(30)) + cad.TIBIA * np.sin(np.radians(15))
print(f'standby foot: radial {tip_reach:.1f} mm from hip, {cad.TIBIA * np.cos(np.radians(15)) - cad.FEMUR * np.sin(np.radians(30)):.1f} mm below hip')
femur_arm = cad.FEMUR * np.cos(np.radians(30)) + cad.TIBIA * np.sin(np.radians(15))
tibia_arm = cad.TIBIA * np.sin(np.radians(15))
load = total / 1000 * g / 3
print(f'static tripod load per foot {load:.2f} N; femur torque {load * femur_arm / 1000:.3f} N.m; tibia torque {load * tibia_arm / 1000:.3f} N.m; stall 6 V {EFFORT} N.m')
