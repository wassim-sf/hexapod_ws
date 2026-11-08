"""Assembly of the hexapod STL parts into URDF link frames. Millimetres.

Part set: refs/emre_hexapod (Robot Lk package). It mixes the 60 mm "Chipo" coxa and femur from
github.com/makeyourpet/hexapod with that project's long tibia and shield. See docs/robot_parameters.md.

Leg link frames at q = 0: x radially outward, z up, y = z x x. The supplied parts are for left-side
legs, so they are placed directly for those; right-side legs are the mirror image (y -> -y).

Everything here is measured from the STLs. measure_geometry.py re-derives it and fails on drift.
"""
import pathlib

import numpy as np
import trimesh

ROOT = pathlib.Path(__file__).resolve().parents[1]
PARTS = ROOT / 'refs/emre_hexapod'

# --- Measured relations ---
SHAFT_OFFSET = 10.25      # shaft from the mounting-hole centre, along the servo: calibration-arm hole vs calibration-ruler pattern
FRAME_PATTERN_CENTER = 71.5          # centre legs, frame holes x = 47.5 .. 95.5
CORNER_PATTERN_CENTER = (55.7, 75.9)  # corner legs, long axis at 45 deg
FRAME_THICKNESS = 16.0
COXA_PIVOT = (3.0, 1.0)   # Left-coxa-996: centre of the horn screw circle
COXA_SERVO_X = 46.0       # Left-coxa-996: hole pattern x = 41 .. 51
COXA_SERVO_Z_CENTER = 14.5           # hole pattern z = -9.5 .. 38.5
COXA = COXA_SERVO_X - COXA_PIVOT[0]  # 43.0, matches COXA_LEN 43 in the project config
FEMUR = 60.0              # Left-femur-996: hub centres
TIBIA_HOLES = ((67.0, 0.5), (59.4, 7.0), (36.2, -36.2), (28.5, -29.8))  # Left-tibia, (x, z)
FOOT_TIP_TIBIA = (12.0, -133.0)      # tip bottom in tibia STL (x, z): see FOOT_T below
# Servo stack along the shaft, derived: the coxa arm gap of 48 mm = 11 + 34 + 3.
FLANGE_TO_TIP = 11.0      # flange face to the end of the shaft and horn seat
FLANGE_TO_BACK = 30.0     # flange face to the wire-side end. Idler ceiling sits here (idler z = 30)
IDLER_CAP = 4.0
RING = 3.0
# Datasheet outline of the servo
SERVO_L, SERVO_W = 40.7, 19.7
FLANGE_L, FLANGE_T = 54.0, 2.5
GEAR_TOP = 8.0            # EST: body height on the shaft side of the flange
HORN_DIA, HORN_T = 25.0, 2.5

COXA_Z_LOCAL = COXA_SERVO_Z_CENTER - SHAFT_OFFSET  # femur axis height in the coxa STL (shaft towards the low end, from the assembly images)
COXA_LOCAL_TO_BASE_Z = -FLANGE_TO_TIP + 10.0       # the coxa lower-arm inner face (z = -10) meets the shaft tip
HIP_Z = COXA_Z_LOCAL + COXA_LOCAL_TO_BASE_Z        # femur axis above the frame underside (base_link z = 0)


def tibia_shaft():
    h = np.array(TIBIA_HOLES)
    upper, lower = (h[0] + h[1]) / 2, (h[2] + h[3]) / 2
    d = (upper - lower) / np.linalg.norm(upper - lower)
    return (upper + lower) / 2 + SHAFT_OFFSET * d, d


SHAFT_T, AXIS_T = tibia_shaft()
_foot = np.array(FOOT_TIP_TIBIA) - SHAFT_T
TIBIA = float(np.hypot(*_foot))
TIBIA_TILT = float(np.arctan2(-_foot[0], -_foot[1]))  # in the mapped frame the foot lies out by this angle before the zero-pose rotation


def transform(rotation, translation=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def rz(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def _about_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# --- STL frame -> link frame, left-side legs ---
COXA_T = transform(np.eye(3), (-COXA_PIVOT[0], -COXA_PIVOT[1], -COXA_Z_LOCAL))
FEMUR_T = transform(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float), (0, 21.0, 0))  # x = x, y = 21 - z, z = y
# Tibia: x -> -x, y -> -4.5 - y, z -> z about the knee, then rotated so the foot hangs straight down.
_flip = np.diag([-1.0, -1.0, 1.0])
_rot = _about_y(np.arctan2(-_foot[0], -_foot[1]))
TIBIA_T = transform(_rot @ _flip, _rot @ (_flip @ -np.array([SHAFT_T[0], 0.0, SHAFT_T[1]]) + np.array([0, -4.5, 0])))
SHIELD_T = TIBIA_T @ transform(np.eye(3), (0, -9.0, 0))   # its 40 mm width centres on the 9 mm tibia bar
FOOT_T = TIBIA_T @ transform(np.eye(3), (11.0, -4.5, -130.0))  # ms-holder, rod and limitter: bar slot x -9.7 -> bar x 1.3; wedge end z 20 -> -110


def _tibia_foot_offset():
    p = TIBIA_T @ np.array([FOOT_TIP_TIBIA[0], -4.5, FOOT_TIP_TIBIA[1], 1.0])
    return p[:3]


def foot_frame_from_tibia():
    return transform(np.eye(3), _tibia_foot_offset())


TIP_T = np.linalg.inv(foot_frame_from_tibia()) @ FOOT_T  # tip in the foot link: origin at the tip bottom


# --- Placement of servos: local x = along the long side from the shaft, z = towards the shaft tip, origin at the flange on the shaft axis ---
def _servo_axes(x, z):
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.column_stack((x, np.cross(z, x), z))


COXA_SERVO_T = transform(_servo_axes((0, 0, 1), (0, 1, 0)), (COXA, 8.0, 0.0))
TIBIA_SERVO_T = TIBIA_T @ transform(_servo_axes((-AXIS_T[0], 0, -AXIS_T[1]), (0, -1, 0)), (SHAFT_T[0], -12.5, SHAFT_T[1]))


def servo_mesh(horn=True, body_only=False):
    """Placeholder MG996R from datasheet dimensions, until the real model is supplied."""
    def box(ext, c):
        b = trimesh.creation.box(extents=ext)
        b.apply_translation(c)
        return b
    x_mid = SHAFT_OFFSET
    z_mid = (GEAR_TOP - FLANGE_TO_BACK) / 2
    parts = [box((SERVO_L, SERVO_W, GEAR_TOP + FLANGE_TO_BACK), (x_mid, 0, z_mid)),
             box((FLANGE_L, SERVO_W, FLANGE_T), (x_mid, 0, -FLANGE_T / 2))]
    if not body_only:
        shaft = trimesh.creation.cylinder(radius=3.0, height=FLANGE_TO_TIP - GEAR_TOP, sections=24)
        shaft.apply_translation((0, 0, (FLANGE_TO_TIP + GEAR_TOP) / 2))
        parts.append(shaft)
        if horn:
            disc = trimesh.creation.cylinder(radius=HORN_DIA / 2, height=HORN_T, sections=32)
            disc.apply_translation((0, 0, FLANGE_TO_TIP - HORN_T / 2))
            parts.append(disc)
    return trimesh.util.concatenate(parts)


def idler_transform(servo_t):
    """The idler caps the servo's wire-side end, with its hole on the shaft axis."""
    # idler: hole (10, 10), ceiling at z = 30, outside towards +z, so flip z onto the servo's back.
    return servo_t @ transform(np.diag([1.0, -1.0, -1.0]), (-10.0, 10.0, -FLANGE_TO_BACK + 30.0))


def ring_transform(servo_t):
    return servo_t @ transform(np.eye(3), (0, 0, -FLANGE_TO_BACK - IDLER_CAP - RING))


_cache = {}


def load(name):
    if name not in _cache:
        mesh = trimesh.load_mesh(PARTS / f'{name}.stl', force='mesh')
        bodies = [b for b in mesh.split(only_watertight=False) if abs(b.volume) > 1.0]
        mesh = trimesh.util.concatenate(bodies) if bodies else mesh
        trimesh.repair.fix_normals(mesh)
        _cache[name] = mesh
    return _cache[name].copy()


def placed(name, t):
    mesh = load(name)
    mesh.apply_transform(t)
    return mesh


# (label, STL name, transform). Quantities are per leg.
COXA_PARTS = [('coxa', 'Left-coxa-996', COXA_T), ('idler', 'idler', idler_transform(COXA_SERVO_T)), ('ring', 'ring', ring_transform(COXA_SERVO_T))]
FEMUR_PARTS = [('femur', 'Left-femur-996', FEMUR_T)]
TIBIA_PARTS = [('tibia', 'Left-tibia', TIBIA_T), ('shield', 'shield', SHIELD_T), ('ms_holder', 'ms-holder', FOOT_T), ('rod', 'rod', FOOT_T),
               ('limitter', 'limitter', FOOT_T), ('idler', 'idler', idler_transform(TIBIA_SERVO_T)), ('ring', 'ring', ring_transform(TIBIA_SERVO_T))]
FOOT_PARTS = [('tip', 'tip', TIP_T)]

# Leg table: id, name, side, yaw in base_link (x forward, y left).
LEGS = [(1, 'front_right', 'right', -45.0), (2, 'center_right', 'right', -90.0), (3, 'rear_right', 'right', -135.0),
        (4, 'front_left', 'left', 45.0), (5, 'center_left', 'left', 90.0), (6, 'rear_left', 'left', 135.0)]


def hip_xy(yaw):
    """Hip position in base_link. STL +y is taken as forward, so x_base = y_stl, y_base = -x_stl."""
    a = abs(yaw)
    if a == 90.0:
        y = np.sign(yaw) * (FRAME_PATTERN_CENTER + SHAFT_OFFSET)
        return np.array([0.0, y])
    s = SHAFT_OFFSET * np.cos(np.radians(45))
    x_stl = CORNER_PATTERN_CENTER[0] + s
    y_stl = CORNER_PATTERN_CENTER[1] + s
    return np.array([np.sign(np.cos(np.radians(yaw))) * y_stl, np.sign(yaw) * x_stl])


# STL body frame -> base_link: rotate so +y is forward, and z = 0 at the frame underside (where the servo flanges sit).
_to_base = rz(-90)
# The frame is modelled upside down: its screw holes open at z = 16, which is really its underside.
FRAME_T = transform(_to_base @ np.diag([-1.0, 1.0, -1.0]), (0, 0, FRAME_THICKNESS))
BOTTOM_COVER_T = transform(_to_base, (0, 0, 16.0))    # pillars (top z = -16) meet the frame underside
TOP_COVER_T = transform(_to_base, (0, 0, FRAME_THICKNESS))
BAR_SHIFT = 7.5                                       # the bar's tabs at y = 32.5 move to the cover holes at y = 40
FLOOR_TOP = -11.5                                     # floor of the bottom cover, from a ray test
FLOOR_BOTTOM = -14.0


def body_parts():
    parts = [('frame', 'frame', FRAME_T), ('bottom_cover', 'bottom-cover-flat', BOTTOM_COVER_T), ('top_cover', 'top-cover4', TOP_COVER_T)]
    for side in (1, -1):
        t = transform(_to_base @ np.diag([1.0, float(side), 1.0]), (0, 0, FLOOR_BOTTOM + 28.0))
        shift = transform(np.eye(3), (0, BAR_SHIFT, 0))
        parts.append((f'battery_bar_{side}', 'battery-bar', t @ shift))
    return parts


def leg_base_transform(yaw):
    xy = hip_xy(yaw)
    return transform(rz(yaw), (xy[0], xy[1], 0.0))


def body_servo_t(yaw):
    """Coxa servo fixed in the body: shaft down, long side inward."""
    return leg_base_transform(yaw) @ transform(_servo_axes((-1, 0, 0), (0, 0, -1)))


def body_with_idlers():
    parts = [placed(n, t) for _, n, t in body_parts()]
    for _, _, _, yaw in LEGS:
        parts.append(placed('idler', idler_transform(body_servo_t(yaw))))
        parts.append(placed('ring', ring_transform(body_servo_t(yaw))))
    return trimesh.util.concatenate(parts)


def merged(parts):
    return trimesh.util.concatenate([placed(n, t) for _, n, t in parts])


def mirror(mesh):
    m = mesh.copy()
    m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
    return m
