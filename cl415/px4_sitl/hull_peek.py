"""quick look at a hull state recording (gz topic -e --json-output of /cl415/hull)

    python3 px4_sitl/hull_peek.py ~/cl415_sim_logs/hull_test_state.jsonl
prints the resistance curve (binned by speed) with the per-element split of submerged volume, planing lift and drag
"""
import json
import sys

import numpy as np

NAMES = ['c0', 'c1', 'c2', 'c3', 'c4', 'c5', 'Lfl', 'Rfl', 'Lwg', 'Rwg']
W = 10.5 * 9.81
rows = []
for line in open(sys.argv[1] if len(sys.argv) > 1 else '/home/test/cl415_sim_logs/hull_test_state.jsonl'):
    if line.startswith('{'):
        try:
            rows.append(json.loads(line)['data'])
        except Exception:
            pass
n = max(len(r) for r in rows)
a = np.array([r for r in rows if len(r) == n])
t, vol, buoy, R, plz, heave, keel, V, awp, vz = a[:, :10].T
ne = (n - 10) // 3
el = a[:, 10:].reshape(len(a), ne, 3)
print(f'{len(a)} samples, {t[0]:.1f} to {t[-1]:.1f} s, {ne} elements, max speed {V.max():.1f} m/s, final speed {np.median(V[-50:]):.1f}')
print('speed  R/W   buoy/W  plane/W  keel   | vol share %        | planing lift / W per element           | drag / W per element')
for v0 in np.arange(0, V.max() + 1, 1.0):
    s = (V >= v0) & (V < v0 + 1)
    if s.sum() < 5:
        continue
    vs = el[s, :, 0].mean(0)
    vshare = 100 * vs / max(vs.sum(), 1e-9)
    pl = el[s, :, 1].mean(0) / W
    dr = el[s, :, 2].mean(0) / W
    print(f'{v0:4.0f}  {np.median(R[s]) / W:5.2f}  {np.median(buoy[s]) / W:5.2f}   {np.median(plz[s]) / W:5.2f}  {np.median(keel[s]):5.3f} | '
          + ' '.join(f'{x:3.0f}' for x in vshare) + ' | ' + ' '.join(f'{x:5.2f}' for x in pl) + ' | ' + ' '.join(f'{x:5.2f}' for x in -dr))
print('elements:', ' '.join(f'{i}:{nm}' for i, nm in enumerate(NAMES[:ne])))
