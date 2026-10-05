# quick orthographic renders with matplotlib (painter's algorithm, flat shading)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

# view name -> (screen x axis, screen y axis, toward-viewer axis), all in REP-103 body frame
VIEWS = {
    'top':   (np.array([0, -1, 0]), np.array([1, 0, 0]), np.array([0, 0, 1])),
    'side':  (np.array([-1, 0, 0]), np.array([0, 0, 1]), np.array([0, 1, 0])),
    'front': (np.array([0, 1, 0]), np.array([0, 0, 1]), np.array([1, 0, 0])),
}


def _iso_axes(az=-135.0, el=25.0):
    az, el = np.radians(az), np.radians(el)
    toward = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.cross(np.array([0, 0, 1.0]), toward)
    right /= np.linalg.norm(right)
    up = np.cross(toward, right)
    return right, up, toward


VIEWS['iso'] = _iso_axes()

AXIS_LABEL = {'top': ('-Y (right wing to the right)', 'X (nose up)'),
              'side': ('-X (nose to the left)', 'Z (up)'),
              'front': ('Y (left wing to the right)', 'Z (up)'),
              'iso': ('', '')}


def draw(ax, parts, view, light=(0.3, 0.4, 0.85), outline=None):
    """parts: list of (trimesh, rgb). outline: optional list of 2d polygons (screen coords) drawn dashed"""
    sx, sy, sz = VIEWS[view]
    L = np.asarray(light, float)
    L /= np.linalg.norm(L)
    polys, cols, depth = [], [], []
    for mesh, rgb in parts:
        tri = mesh.vertices[mesh.faces]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nn = np.linalg.norm(n, axis=1)
        ok = nn > 1e-15
        tri, n = tri[ok], n[ok] / nn[ok, None]
        # light in screen space so every view is lit the same way
        nl = np.abs(n @ (L[0] * sx + L[1] * sy + L[2] * sz))
        shade = 0.35 + 0.65 * nl
        p2 = np.stack([tri @ sx, tri @ sy], axis=-1)
        polys.append(p2)
        cols.append(np.clip(np.asarray(rgb)[None, :] * shade[:, None], 0, 1))
        depth.append((tri @ sz).mean(axis=1))
    P = np.concatenate(polys)
    C = np.concatenate(cols)
    D = np.concatenate(depth)
    order = np.argsort(D)
    pc = PolyCollection(P[order], facecolors=C[order], edgecolors=C[order], linewidths=0.15)
    ax.add_collection(pc)
    allp = P.reshape(-1, 2)
    if outline is not None:
        for poly in outline:
            ax.plot(poly[:, 0], poly[:, 1], '--', color='crimson', lw=0.8)
            allp = np.vstack([allp, poly])
    lo, hi = allp.min(0), allp.max(0)
    pad = 0.04 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect('equal')
    if view != 'iso':
        ax.set_xlabel(AXIS_LABEL[view][0] + ' [m]')
        ax.set_ylabel(AXIS_LABEL[view][1] + ' [m]')
        ax.grid(True, lw=0.3, alpha=0.5)
        ax.plot(0, 0, '+', color='k', ms=10, mew=1.2)
    else:
        ax.set_axis_off()
    ax.set_title(view)


def silhouette(mesh, view):
    """2d outline of a mesh in a view (union of projected triangles)"""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    sx, sy, _ = VIEWS[view]
    tri = mesh.vertices[mesh.faces]
    p2 = np.stack([tri @ sx, tri @ sy], axis=-1)
    d1, d2 = p2[:, 1] - p2[:, 0], p2[:, 2] - p2[:, 0]
    keep = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) > 1e-12
    shapes = [Polygon(t) for t in p2[keep]]
    u = unary_union(shapes).buffer(0)
    geoms = getattr(u, 'geoms', [u])
    out = []
    for g in geoms:
        out.append(np.asarray(g.exterior.coords))
    return out


def save_views(parts, path, views=('top', 'side', 'front', 'iso'), title=None, outlines=None, dpi=170):
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    for ax, v in zip(axs.ravel(), views):
        draw(ax, parts, v, outline=None if outlines is None else outlines.get(v))
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def save_single(parts, path, view, title=None, outline=None, dpi=170, figsize=(10, 7)):
    fig, ax = plt.subplots(figsize=figsize)
    draw(ax, parts, view, outline=outline)
    if title:
        ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
