"""pairwise interference check of the exported step parts (run after build_cad.py)"""
import itertools
import json
import os

import cadquery as cq

CAD = os.path.dirname(os.path.abspath(__file__))
geo = json.load(open(os.path.join(CAD, 'geometry.json')))
names = [n for n, p in geo['parts'].items() if p['link'] is not None]
# importStep reads the metre files into a mm session, so scale back
S = {n: cq.importers.importStep(os.path.join(CAD, 'parts', f'{n}.step')).val().scale(0.001) for n in names}
rows = []
where = {}
for a, b in itertools.combinations(names, 2):
    ba, bb = S[a].BoundingBox(), S[b].BoundingBox()
    if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
            or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
        continue
    inter = S[a].intersect(S[b])
    v = 0.0
    if inter.Solids():
        vs, tr = inter.tessellate(0.0002, 0.1)
        if tr:
            import numpy as np, trimesh
            v = abs(float(trimesh.Trimesh(np.array([q.toTuple() for q in vs]), np.array(tr)).volume))
            bb = inter.BoundingBox()
            where[(a, b)] = [round(bb.xmin, 3), round(bb.xmax, 3), round(bb.ymin, 3), round(bb.ymax, 3), round(bb.zmin, 3), round(bb.zmax, 3)]
    rows.append((a, b, v))
lines = ['# CAD interference check', '', 'Pairs whose bounding boxes overlap, with the volume their solids share.', '',
         '| part A | part B | shared volume [mm^3] |', '|---|---|---|']
for a, b, v in sorted(rows, key=lambda r: -r[2]):
    lines.append(f'| {a} | {b} | {v * 1e9:.3f} |' + (f' bbox {where[(a, b)]}' if v > 1e-9 else ''))
worst = max((r[2] for r in rows), default=0.0)
lines += ['', f'Largest overlap: {worst * 1e9:.3f} mm^3.']
open(os.path.join(CAD, 'interference.md'), 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines))
