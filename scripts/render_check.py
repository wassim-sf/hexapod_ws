#!/usr/bin/env python3
"""Render the assembled CAD parts from their link transforms, for comparison with the RookiDroid renders."""
import sys, pathlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import hexapod_cad as cad

COL = dict(body='#c96a6a', coxa='#6fc9c4', femur='#7fd07f', tibia='#6a6ac9', foot='#333333')

def small(m, n):
    return m.simplify_quadric_decimation(face_count=n) if len(m.faces) > n else m

def assemble(q=(0, 30, -15), legs=cad.LEGS, faces=1500):
    items = [(small(m, 2500), COL['body']) for _, _, m in cad.placed(cad.body_parts())]
    sets = [(cad.COXA_PARTS, 0, 'coxa'), (cad.FEMUR_PARTS, 1, 'femur'), (cad.TIBIA_PARTS, 2, 'tibia'), (cad.FOOT_PARTS, 3, 'foot')]
    for _, _, side, yaw in legs:
        tfs = cad.leg_transforms(yaw, q)
        for parts, i, c in sets:
            for _, _, m in cad.placed(parts):
                mm = small(m, faces)
                if side == 'left':
                    mm = cad.mirror(mm)
                mm = mm.copy(); mm.apply_transform(tfs[i]); items.append((mm, COL[c]))
    return items

def draw(items, out, views, lim=210):
    fig = plt.figure(figsize=(6 * len(views), 6))
    for k, (e, a) in enumerate(views):
        ax = fig.add_subplot(1, len(views), k + 1, projection='3d')
        L = np.array([0.4, 0.3, 0.87]); polys = []; cols = []
        for m, c in items:
            t = m.triangles; n = m.face_normals
            s = 0.45 + 0.55 * np.clip(n @ L, 0, 1)
            base = np.array(matplotlib.colors.to_rgb(c))
            polys.append(t); cols.append(s[:, None] * base)
        ax.add_collection3d(Poly3DCollection(np.vstack(polys), facecolors=np.vstack(cols), edgecolors='none'))
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim * 0.7 + 30, lim * 0.7 + 30)
        ax.set_box_aspect((1, 1, 0.7)); ax.view_init(e, a); ax.set_xlabel('x fwd'); ax.set_ylabel('y left')
    plt.tight_layout(); plt.savefig(out, dpi=95); plt.close(fig)

if __name__ == '__main__':
    out = sys.argv[1]
    q = tuple(float(v) for v in sys.argv[2:5]) if len(sys.argv) > 4 else (0, 0, 0)
    draw(assemble(q), out, [(90, -90), (25, -60)])
