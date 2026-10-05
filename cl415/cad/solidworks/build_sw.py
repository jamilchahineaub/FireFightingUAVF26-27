"""solidworks parts, assembly and mass properties for the cl415 (run after mass/mass_props.py plan)

for every entry in mass/plan.json:
  import the cadquery step (3d interconnect off, so it comes in as a plain solid), set MKS units,
  set the part density to budget mass / solidworks volume, colour it, tag it with custom properties,
  save cad/solidworks/parts/<name>.SLDPRT and read its mass properties back
then cad/solidworks/cl415_assembly.SLDASM with every part fixed at the origin (the step files already
sit in the aircraft frame) and two configurations, Full and Empty (water suppressed), and the
assembly mass properties of both. results: cad/solidworks/sw_mass_props.json

frame: the solidworks model frame is REP-103 (x fwd, y left, z up, origin at the wing le root).
inertia is stored in tensor form (products negated from what solidworks reports).

run from the project root with solidworks open:  .venv/Scripts/python cad/solidworks/build_sw.py
(--asm-only rebuilds just the assembly from the saved parts)
"""
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'sw'))
import sw_api as S  # noqa: E402
from sw_tlb import C  # noqa: E402

PLAN = json.load(open(os.path.join(ROOT, 'mass', 'plan.json')))
PART_DIR = os.path.join(HERE, 'parts')
ASM_PATH = os.path.join(HERE, 'cl415_assembly.SLDASM')
os.makedirs(PART_DIR, exist_ok=True)


def open_docs(sw):
    out = []
    d = sw.GetFirstDocument()
    while d is not None:
        out.append(d)
        d = d.GetNext()
    return out


def close_if_open(sw, path_or_title):
    # by path, or by title for unsaved imports (getopendocumentbyname misses those)
    key = os.path.normcase(os.path.normpath(path_or_title))
    title = os.path.basename(path_or_title).lower()
    for d in open_docs(sw):
        p = d.GetPathName()
        if (p and os.path.normcase(os.path.normpath(p)) == key) or d.GetTitle().lower() == title:
            sw.CloseDoc(d.GetTitle())


def read_props(model):
    p = S.massprops(model)
    return {'mass': p['m'], 'cg': p['cg'].tolist(), 'I_cg': S.tensor_from_sw(p['I_cg']).tolist(),
            'volume': p['V'], 'density': p['density']}


def set_colour(model, rgb, transparency=0.0):
    model.MaterialPropertyValues = S.vr8([rgb[0], rgb[1], rgb[2], 0.6, 0.9, 0.4, 0.3, transparency, 0.0])


def build_part(sw, item):
    src = os.path.normpath(os.path.join(ROOT, item['step']))
    dst = os.path.join(PART_DIR, item['name'] + '.SLDPRT')
    close_if_open(sw, dst)
    close_if_open(sw, item['name'] + '.SLDPRT')
    err = S.vref_i4()
    model = sw.LoadFile4(src, 'r', S.vnull(), err)
    if model is None:
        raise RuntimeError(f'import failed ({err.value}) for {src}')
    model = S.wrap(model)
    nb = len(model.GetBodies2(C('swSolidBody'), False) or ())
    model.Extension.SetUserPreferenceInteger(C('swUnitSystem'), 0, C('swUnitSystem_MKS'))
    vol = S.massprops(model)['V']
    rho = item['mass'] / vol
    if not model.Extension.SetUserPreferenceDouble(C('swMaterialPropertyDensity'), 0, rho):
        raise RuntimeError(f'could not set density on {item["name"]}')
    set_colour(model, item['rgb'], 0.6 if item['kind'] == 'water' else 0.0)
    S.set_props(model, {'SIM_LINK': item['link'], 'TARGET_MASS_KG': f'{item["mass"]:.4f}',
                        'KIND': item['kind'], 'DESCRIPTION': item['desc'], 'SOURCE_STEP': item['step'],
                        'BUILT_BY': 'cad/solidworks/build_sw.py'})
    model.ForceRebuild3(False)
    props = read_props(model)
    props['solid_bodies'] = nb
    S.save_as(model, dst)
    sw.CloseDoc(model.GetTitle())
    return dst, props


def build_assembly(sw, part_paths):
    close_if_open(sw, ASM_PATH)
    if os.path.exists(ASM_PATH):
        os.remove(ASM_PATH)
    asm = S.new_doc(sw, 'assembly')
    asm.Extension.SetUserPreferenceInteger(C('swUnitSystem'), 0, C('swUnitSystem_MKS'))
    S.save_as(asm, ASM_PATH)
    comps = {}
    opened = []
    for name, path in part_paths.items():
        if sw.GetOpenDocumentByName(path) is None:
            S.open_doc(sw, path, 'part')
            opened.append(path)
        e = S.vref_i4()
        sw.ActivateDoc3(asm.GetTitle(), False, 0, e)
        comp = asm.AddComponent5(path, 0, '', False, '', 0.0, 0.0, 0.0)
        if comp is None:
            raise RuntimeError(f'AddComponent5 failed for {path}')
        comp.Transform2 = S.tf_to_sw(sw, np.eye(4))      # addcomponent5 centres the part, put its origin back
        asm.ClearSelection2(True)
        comp.Select4(False, S.vnull(), False)
        asm.FixComponent()
        asm.ClearSelection2(True)
        comps[name] = comp
    for path in opened:
        close_if_open(sw, path)
    cm = asm.ConfigurationManager
    cm.ActiveConfiguration.Name = 'Full'
    cm.AddConfiguration2('Empty', 'water tank empty (water_3L suppressed)', '', 0, '', '', True)
    asm.ShowConfiguration2('Empty')
    water = S.wrap(asm).GetComponentByName(comps['water_3L'].Name2)     # handle of the active config
    water.SetSuppression2(C('swComponentSuppressed'))
    asm.ForceRebuild3(False)
    if not water.IsSuppressed():
        raise RuntimeError('could not suppress water_3L in the Empty configuration')
    out = {}
    for cfg in ('Full', 'Empty'):
        asm.ShowConfiguration2(cfg)
        asm.ForceRebuild3(False)
        out[cfg.lower()] = read_props(asm)
    asm.ShowConfiguration2('Full')
    S.save_as(asm, ASM_PATH)
    try:
        screenshot(sw, asm, os.path.join(HERE, 'cl415_assembly.png'))
    except RuntimeError as ex:
        print('  screenshot skipped:', ex)
    return asm, out


def screenshot(sw, model, png):
    """iso view with the model z axis up (solidworks' named views assume y up), saved as png.
    the png comes from the active window, so the document is activated first"""
    import math
    e = S.vref_i4()
    sw.ActivateDoc3(model.GetTitle(), False, 0, e)
    az, el = math.radians(-135), math.radians(25)
    toward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    right = np.cross([0, 0, 1.0], toward)
    right /= np.linalg.norm(right)
    up = np.cross(toward, right)
    Rv = np.array([right, up, toward]).T      # solidworks wants the model axes as rows, in view coordinates
    t = sw.GetMathUtility().CreateTransform(S.vr8(list(Rv.reshape(9)) + [0, 0, 0, 1.0, 0, 0, 0]))
    model.ActiveView.Orientation3 = t
    model.ViewZoomtofit2()
    model.GraphicsRedraw2()
    S.save_as(model, png)


def main():
    t0 = time.time()
    sw, started = S.connect()
    rev = sw.RevisionNumber()
    print('solidworks', rev, '(started)' if started else '(attached)')
    ic = sw.GetUserPreferenceToggle(C('swMultiCAD_Enable3DInterconnect'))
    sw.SetUserPreferenceToggle(C('swMultiCAD_Enable3DInterconnect'), False)
    res = {'solidworks_revision': rev, 'frame': 'REP-103, origin wing LE root, metres',
           'inertia_convention': 'tensor (products = -integral)', 'parts': {}}
    paths = {}
    close_if_open(sw, ASM_PATH)     # an open assembly keeps its parts loaded, close it before rebuilding them
    try:
        for item in PLAN['parts']:
            t = time.time()
            if '--asm-only' in sys.argv:     # reuse the saved parts, just re-read them
                path = os.path.join(PART_DIR, item['name'] + '.SLDPRT')
                close_if_open(sw, path)
                model = S.open_doc(sw, path, 'part')
                props = read_props(model)
                sw.CloseDoc(model.GetTitle())
            else:
                path, props = build_part(sw, item)
            paths[item['name']] = path
            res['parts'][item['name']] = props
            print(f"  {item['name']:24s} m {props['mass']:7.4f} kg  rho {props['density']:9.1f}  "
                  f"V {props['volume'] * 1e6:9.1f} cm3  cg {np.round(props['cg'], 4).tolist()}  {time.time() - t:4.1f}s")
        asm, tot = build_assembly(sw, paths)
        res['assembly'] = tot
        res['assembly_path'] = os.path.relpath(ASM_PATH, ROOT).replace('\\', '/')
        for k, v in tot.items():
            print(f'  assembly {k:5s}: m {v["mass"]:.4f} kg, cg {np.round(v["cg"], 4).tolist()}')
    finally:
        sw.SetUserPreferenceToggle(C('swMultiCAD_Enable3DInterconnect'), ic)
    json.dump(res, open(os.path.join(HERE, 'sw_mass_props.json'), 'w'), indent=1)
    print(f'done in {time.time() - t0:.0f}s')


if __name__ == '__main__':
    main()
