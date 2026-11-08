"""Assembly of the RookiDroid Mochi STL parts into URDF link frames.

Every number here comes from measuring the STL files. measure_geometry.py re-derives
them and fails if they drift. Units are millimetres in this module.

Link frames, right-side leg, neutral pose (coxa radial, femur level, tibia vertical):
  x = radially outward, z = up, y = z cross x. Servo horns are on +y.
Left-side legs are the mirror image (y -> -y). This matches the README's mirrored
joints/feet and the top-view CAD render in robots/mochi/images/calibration_2.jpg.
"""
import pathlib
import numpy as np
import trimesh

MOCHI = pathlib.Path(__file__).resolve().parents[1] / 'refs/rookidroid_hexapod/robots/mochi/3d print'

# Measured stack (mm)
HIP_RADIUS = 81.80        # body_base: six 7 mm bearing seats on this radius, 60 deg apart
ARM_HEIGHT = 28.80        # base 5.0 + servo_side 20.8 + top 3.0
CROSS_HALF = 20.765       # joint_cross half-extent; joint plates seat against its edge
HIP_Z = CROSS_HALF        # coxa-femur plane above the base underside (plate touches base)
COXA = 36.00              # 2 x 18.0: joint plates, pivot hole to cross hole
FEMUR = 43.60             # leg_bottom: bearing seat centres 10.5 and 54.1
TIBIA = 83.46             # 20.0 (ground start) + 56.46 (notch floor) + 7.0 (tip beyond floor)
FOOT_WIDTH = 39.50        # foot_ground width; foot plates seat against its edges
BLOCK_LOW = -CROSS_HALF   # femur block outer face on the pin side
BLOCK_HIGH = BLOCK_LOW + ARM_HEIGHT
TIP_Y = BLOCK_LOW + FOOT_WIDTH / 2.0  # lateral foot offset of the assembled parts (-1.0 mm)
BATTERY_PLATE = 3.0       # body_battery plate thickness
HEAD_Z = ARM_HEIGHT + BATTERY_PLATE
HEAD_HEIGHT = 58.0
BODY_HEIGHT = HEAD_Z + HEAD_HEIGHT

# Part axes -> link axes
I3 = np.eye(3)
RZ180 = np.diag([-1., -1., 1.])
RY180 = np.diag([-1., 1., -1.])
RX180 = np.diag([1., -1., -1.])
RXP90 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)   # y->z, z->-y
RXM90 = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)   # y->-z, z->y
FOOT = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]], float)  # x->-z, y->-y, z->-x
CROSS = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)   # x->-z, y->y, z->x


def rz(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def tf(rot, part_pt, link_pt):
    """Homogeneous transform: rotate the part, then put part_pt at link_pt."""
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = np.asarray(link_pt, float) - rot @ np.asarray(part_pt, float)
    return t


def pre(rot, t):
    """Apply a further rotation (3x3) to transform t."""
    m = np.eye(4)
    m[:3, :3] = rot
    return m @ t


# (name, file, transform) per link, right-side leg
C = CROSS_HALF
COXA_PARTS = [
    ('joint_top_hip', 'joint/joint_top.STL', tf(RY180, (23, 7.5, 3.4), (0, 0, C))),
    ('joint_bottom_hip', 'joint/joint_bottom.STL', tf(RZ180, (23, 7.5, 3.64), (0, 0, -C))),
    ('joint_top_femur', 'joint/joint_top.STL', tf(RXP90, (23, 7.5, 3.4), (COXA, C, 0))),
    ('joint_bottom_femur', 'joint/joint_bottom.STL', tf(RXM90, (23, 7.5, 3.64), (COXA, -C, 0))),
    ('joint_cross', 'joint/joint_cross.STL', tf(CROSS, (58.505, 24.375, 6.0), (COXA / 2, 0, 0))),
]
FEMUR_PARTS = [
    ('leg_bottom', 'leg/leg_bottom.STL', tf(I3, (10.5, 0.28, 11.23), (0, BLOCK_LOW, 0))),
    ('leg_top', 'leg/leg_top.STL', tf(RXP90, (10.5, 28.45, 46.68), (0, BLOCK_HIGH, 0))),
    ('leg_side_up', 'leg/leg_side.STL', tf(I3, (10.5, 0, 0), (0, BLOCK_LOW + 5.0, 7.2))),
    ('leg_side_down', 'leg/leg_side.STL', tf(RX180, (10.5, 0, 0), (0, BLOCK_HIGH - 3.0, -7.2))),
]
TIBIA_PARTS = [
    ('foot_bottom', 'foot/foot_bottom.STL', tf(FOOT, (8, 35.9, 8), (0, BLOCK_LOW, 0))),
    ('foot_top', 'foot/foot_top.STL', tf(FOOT, (14.5, 3.4, 7.5), (0, BLOCK_LOW + FOOT_WIDTH, 0))),
    ('foot_ground', 'foot/foot_ground.STL', tf(FOOT, (0, 19.75, 2), (0, TIP_Y, -20))),
]
# foot frame origin = tip extreme, on the planar leg line
FOOT_PARTS = [
    ('foot_tip', 'foot/foot_tip.STL', tf(FOOT, (28.96, 19.75, 6), (0, TIP_Y, 0))),
]


def body_parts():
    """Body parts in base_link: x forward, y left, z up, origin under the centre at the base underside.

    The firmware frame has X right and Y forward, which is Rz(-90) from ROS.
    """
    to_ros = rz(-90)
    parts = [
        ('body_base', 'body/body_base.STL', pre(to_ros, tf(RXP90, (91.8, 0.32, 83.637), (0, 0, 0)))),
        ('body_top', 'body/body_top.STL', pre(to_ros, tf(I3, (91.8, 83.637, 0.11), (0, 0, 25.8)))),
        ('body_battery', 'body/body_battery.3mf', pre(to_ros, tf(I3, (128, 128, 0), (0, 0, ARM_HEIGHT)))),
        ('body_head', 'body/body_head.STL', pre(to_ros, tf(I3, (66, 66, 0), (0, 0, HEAD_Z)))),
    ]
    for k in range(6):
        parts.append((f'body_side_{k}', 'body/body_side.STL',
                      pre(to_ros @ rz(60 * k), tf(I3, (18, 2, 0), (0, -62.04, 1)))))
        arm = pre(to_ros @ rz(60 * k), np.eye(4))
        parts.append((f'body_servo_side_{k}a', 'body/body_servo_side.STL',
                      arm @ tf(I3, (0, 2, 0), (0, 9.2, 5))))
        parts.append((f'body_servo_side_{k}b', 'body/body_servo_side.STL',
                      arm @ tf(RX180, (0, 2, 20.8), (0, -9.2, 5))))
    return parts


_cache = {}


def load_part(rel):
    if rel not in _cache:
        loaded = trimesh.load(MOCHI / rel)
        if isinstance(loaded, trimesh.Scene):
            mesh = list(loaded.geometry.values())[0].copy()
            mesh.apply_transform(loaded.graph[list(loaded.graph.nodes_geometry)[0]][0])
        else:
            mesh = loaded
        _cache[rel] = mesh
    return _cache[rel].copy()


def placed(parts):
    """Return [(name, rel, mesh in link frame)]."""
    out = []
    for name, rel, t in parts:
        m = load_part(rel)
        m.apply_transform(t)
        out.append((name, rel, m))
    return out


def merged(parts):
    return trimesh.util.concatenate([m for _, _, m in placed(parts)])


def mirror(mesh):
    m = mesh.copy()
    m.apply_transform(np.diag([1., -1., 1., 1.]))
    return m


# Leg table in base_link. Order follows software/path_tool/robots/mochi.json.
LEGS = [
    (1, 'front_right', 'right', -30.0),
    (2, 'center_right', 'right', -90.0),
    (3, 'rear_right', 'right', -150.0),
    (4, 'front_left', 'left', 30.0),
    (5, 'center_left', 'left', 90.0),
    (6, 'rear_left', 'left', 150.0),
]


def t_rz(deg, xyz=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rz(deg)
    t[:3, 3] = xyz
    return t


def t_axis_y(deg, xyz=(0, 0, 0)):
    """Rotation about link -y by +deg (positive femur lifts, positive tibia swings out)."""
    a = np.radians(deg)
    t = np.eye(4)
    t[:3, :3] = np.array([[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]])
    t[:3, 3] = xyz
    return t


def leg_transforms(yaw, q=(0, 0, 0)):
    """World transforms of coxa, femur, tibia and foot frames for a leg (q in degrees)."""
    y = np.radians(yaw)
    hip = (HIP_RADIUS * np.cos(y), HIP_RADIUS * np.sin(y), HIP_Z)
    tc = t_rz(yaw, hip) @ t_rz(q[0])
    tf_ = tc @ t_axis_y(q[1], (COXA, 0, 0))
    tt = tf_ @ t_axis_y(q[2], (FEMUR, 0, 0))
    tfoot = tt @ t_rz(0, (0, 0, -TIBIA))
    return tc, tf_, tt, tfoot
