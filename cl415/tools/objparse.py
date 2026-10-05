# tiny obj reader, keeps the "o" groups apart
import numpy as np
import trimesh


def read_obj_groups(path):
    verts = []
    groups = {}
    order = []
    cur = None
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            if line.startswith('v '):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith('o ') or line.startswith('g '):
                cur = line.split(None, 1)[1].strip()
                if cur not in groups:
                    groups[cur] = []
                    order.append(cur)
            elif line.startswith('f '):
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split('/')[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                if cur is None:
                    cur = 'default'
                    groups[cur] = []
                    order.append(cur)
                for k in range(1, len(idx) - 1):
                    groups[cur].append((idx[0], idx[k], idx[k + 1]))
    V = np.asarray(verts, dtype=float)
    out = {}
    for name in order:
        F = np.asarray(groups[name], dtype=np.int64)
        if len(F) == 0:
            continue
        used, inv = np.unique(F, return_inverse=True)
        out[name.replace('.mesh', '')] = trimesh.Trimesh(V[used], inv.reshape(-1, 3), process=False)
    return V, out
