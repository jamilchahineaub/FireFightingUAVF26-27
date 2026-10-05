"""solidworks com helpers (adapted from the sub5rr build scripts)

late-bound pywin32 (win32com.client.dynamic) so part/assembly methods resolve on
whatever interface the object really implements. api units are always SI (m, kg, rad).

frames: for the cl415 the solidworks model frame is the REP-103 body frame itself
(x fwd, y left, z up, origin at the wing le root), because the step files are written
that way. every transform below is a 4x4 column-vector matrix (p_parent = M @ p_child);
solidworks' own row-vector arrays go through tf_from_sw / tf_to_sw.
"""
import os
import time

import numpy as np
import pythoncom
from win32com.client import VARIANT, dynamic

from sw_tlb import C, ENUMS, IFACES, verify

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SW_DIR = os.path.join(os.path.dirname(ROOT), 'cad', 'solidworks')

# com plumbing
# solidworks' IDispatch gives no type info (GetTypeInfo -> element not found), so
# pywin32 probes every attribute by invoking it with no arguments: zero-arg methods
# run on attribute access and some methods with optional args (CreateTransform)
# run with garbage. flag every name the installed typelib declares as a method or
# a parameterized get, unless the same name is a plain property somewhere else.
def _callable_names():
    call, g0 = set(), set()
    for mem in IFACES.values():
        for name, sigs in mem.items():
            for s in sigs:
                if s['kind'] == 'method' or (s['kind'] == 'get' and s['nparams'] > 0):
                    call.add(name)
                elif s['kind'] == 'get':
                    g0.add(name)
    return call - g0, call & g0


METHOD_NAMES, AMBIGUOUS_NAMES = _callable_names()


class SWDispatch(dynamic.CDispatch):
    def __getattr__(self, attr):
        if attr in METHOD_NAMES and attr not in self._olerepr_.mapFuncs:
            try:
                self._FlagAsMethod(attr)
            except pythoncom.com_error:
                raise AttributeError(attr)
        return dynamic.CDispatch.__getattr__(self, attr)

    def _wrap_dispatch_(self, ob, userName=None, returnCLSID=None, UnicodeToString=None):
        return dynamic.Dispatch(ob, userName, createClass=SWDispatch)


def wrap(ob):
    return dynamic.Dispatch(ob, createClass=SWDispatch)


def vnull():
    return VARIANT(pythoncom.VT_DISPATCH, None)


def vref_i4(v=0):
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, v)


def vref_bool(v=False):
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_BOOL, v)


def vr8(seq):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [float(x) for x in seq])


def vdisp_arr(seq):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, list(seq))


def vstr_arr(seq):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_BSTR, list(seq))


def connect(visible=True, timeout=600):
    """attach to a running solidworks or start one"""
    pythoncom.CoInitialize()
    try:
        ob = pythoncom.GetActiveObject('SldWorks.Application')
        started = False
    except pythoncom.com_error:
        ob = pythoncom.CoCreateInstance('SldWorks.Application', None, pythoncom.CLSCTX_LOCAL_SERVER,
                                        pythoncom.IID_IDispatch)
        started = True
    sw = wrap(ob.QueryInterface(pythoncom.IID_IDispatch))
    t0 = time.time()
    while True:
        try:
            if sw.StartupProcessCompleted:
                break
        except pythoncom.com_error:
            pass
        if time.time() - t0 > timeout:
            raise TimeoutError('solidworks did not finish starting')
        time.sleep(1.0)
    sw.Visible = visible
    # no dimension popups while scripting
    sw.SetUserPreferenceToggle(C('swInputDimValOnCreate'), False)
    return sw, started


def sw_info(sw):
    info = dict(revision=sw.RevisionNumber())
    base, cur, hot = vref_str(), vref_str(), vref_str()
    try:
        sw.GetBuildNumbers2(base, cur, hot)
        info['build'] = dict(base=base.value, current=cur.value, hotfixes=hot.value)
    except pythoncom.com_error as ex:
        info['build'] = f'n/a ({ex})'
    lic = sw.GetCurrentLicenseType()
    names = {v: k for k, v in ENUMS['swLicenseType_e'].items()}
    info['license_type'] = (lic, names.get(lic, '?'))
    return info


# documents
def new_doc(sw, kind='part'):
    pref = {'part': 'swDefaultTemplatePart', 'assembly': 'swDefaultTemplateAssembly'}[kind]
    tpl = sw.GetUserPreferenceStringValue(C(pref))
    if not tpl or not os.path.exists(tpl):
        raise FileNotFoundError(f'default {kind} template not found: {tpl!r}')
    model = sw.NewDocument(tpl, 0, 0, 0)
    if model is None:
        # a long session with many stale documents sometimes refuses; clear memory and retry once
        sw.CloseAllDocuments(True)
        model = sw.NewDocument(tpl, 0, 0, 0)
    if model is None:
        raise RuntimeError(f'NewDocument failed for {tpl}')
    set_mmgs(model)
    return model


def set_mmgs(model):
    model.Extension.SetUserPreferenceInteger(C('swUnitSystem'), 0, C('swUnitSystem_MMGS'))


def open_doc(sw, path, kind=None, config=''):
    kind = kind or ('assembly' if path.lower().endswith('.sldasm') else 'part')
    t = C('swDocASSEMBLY') if kind == 'assembly' else C('swDocPART')
    err, warn = vref_i4(), vref_i4()
    model = sw.OpenDoc6(path, t, C('swOpenDocOptions_Silent'), config, err, warn)
    if model is None:
        # stale in-memory copies (seen as swFileCriticalDataRepairError) clear after a close-all
        sw.CloseAllDocuments(True)
        err, warn = vref_i4(), vref_i4()
        model = sw.OpenDoc6(path, t, C('swOpenDocOptions_Silent'), config, err, warn)
    if model is None:
        raise RuntimeError(f'OpenDoc6 failed ({err.value}) for {path}')
    return model


def restart(sw=None, timeout=600):
    """quit and relaunch solidworks (frees handles after long build runs)"""
    import time
    if sw is not None:
        try:
            sw.CloseAllDocuments(True)
            sw.ExitApp()
        except pythoncom.com_error:
            pass
        time.sleep(8)
    return connect(timeout=timeout)[0]


def save_as(model, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    err, warn = vref_i4(), vref_i4()
    ok = model.Extension.SaveAs3(path, C('swSaveAsCurrentVersion'), C('swSaveAsOptions_Silent'),
                                 vnull(), vnull(), err, warn)
    if not ok:
        raise RuntimeError(f'SaveAs3 failed err={err.value} warn={warn.value} for {path}')
    return path


def close(sw, model):
    sw.CloseDoc(model.GetTitle())


def std_planes(model):
    """names of the three default planes (front, top, right) whatever the ui language"""
    out = []
    f = model.FirstFeature()
    while f is not None and len(out) < 3:
        if f.GetTypeName2() == 'RefPlane':
            out.append(f.Name)
        f = f.GetNextFeature()
    return out


def last_feature(model):
    return model.FeatureByPositionReverse(0)


def select(model, name, typ, append=False, mark=0):
    ok = model.Extension.SelectByID2(name, typ, 0, 0, 0, append, mark, vnull(), 0)
    if not ok:
        raise RuntimeError(f'could not select {typ} {name!r}')
    return ok


# sketch + features (all lengths in metres)
def sketch_on(model, plane_name):
    model.ClearSelection2(True)
    select(model, plane_name, 'PLANE')
    sm = model.SketchManager
    sm.InsertSketch(True)
    sm.AddToDB = True           # no snapping / inference
    sm.DisplayWhenAdded = False
    return sm


def close_sketch(model, name=None):
    sm = model.SketchManager
    sm.AddToDB = False
    sm.DisplayWhenAdded = True
    sm.InsertSketch(True)
    f = last_feature(model)
    if name:
        f.Name = name
    return f


def extrude_blind(model, sketch_feat, depth, name=None, flip=False, midplane=False):
    model.ClearSelection2(True)
    sketch_feat.Select2(False, 0)
    fm = model.FeatureManager
    t1 = C('swEndCondMidPlane') if midplane else C('swEndCondBlind')
    feat = fm.FeatureExtrusion3(True, False, flip, t1, 0, depth, 0.0,
                                False, False, False, False, 0.0, 0.0,
                                False, False, False, False,
                                True, True, True,
                                C('swStartSketchPlane'), 0.0, False)
    if feat is None:
        raise RuntimeError(f'extrusion failed for {sketch_feat.Name}')
    if name:
        feat.Name = name
    return feat


def set_material(model, name='6061 Alloy', db='SOLIDWORKS Materials'):
    cfg = model.ConfigurationManager.ActiveConfiguration.Name
    model.SetMaterialPropertyName2(cfg, db, name)
    return model.GetMaterialPropertyName2(cfg, vref_str())


def vref_str(v=''):
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_BSTR, v)


def coord_sys_numeric(model, name, origin_m, angles_rad=(0.0, 0.0, 0.0)):
    """coordinate system from numbers (position in the model frame, rotations per api)"""
    a = [float(x) % (2 * np.pi) for x in angles_rad]
    use_rot = any(abs(x) > 1e-12 for x in a)
    f = model.FeatureManager.CreateCoordinateSystemUsingNumericalValues(
        True, float(origin_m[0]), float(origin_m[1]), float(origin_m[2]), use_rot, a[0], a[1], a[2])
    if f is None:
        raise RuntimeError(f'coordinate system {name} not created')
    f.Name = name
    return f


def put_indexed(obj, name, *args):
    """set a parameterized property, e.g. IEquationMgr.Equation(i) = text"""
    dispid = obj._oleobj_.GetIDsOfNames(name)
    return obj._oleobj_.Invoke(dispid, 0, pythoncom.DISPATCH_PROPERTYPUT, 0, *args)


# equations / global variables
def add_equation(model, text, solve=True):
    """Add3 needs a multi-configuration part; Add2 covers the single-config case"""
    eq = model.GetEquationMgr()
    i = eq.Add3(-1, text, solve, C('swAllConfiguration'), vnull())
    if i < 0:
        i = eq.Add2(-1, text, solve)
    if i < 0:
        raise RuntimeError(f'equation rejected: {text}')
    return i


def find_equation(model, lhs):
    eq = model.GetEquationMgr()
    key = f'"{lhs}"'
    for i in range(eq.GetCount()):
        if eq.Equation(i).replace(' ', '').startswith(key + '='):
            return i
    return -1


def set_equation(model, lhs, rhs, solve=True):
    """create or replace '"lhs" = rhs' (rhs already formatted)"""
    text = f'"{lhs}" = {rhs}'
    i = find_equation(model, lhs)
    if i < 0:
        return add_equation(model, text, solve)
    eq = model.GetEquationMgr()
    # SetEquationAndConfigurationOption returns -1 on single-config parts; the
    # indexed property put works everywhere, so use it and check the read-back
    put_indexed(eq, 'Equation', i, text)
    if eq.Equation(i).replace(' ', '') != text.replace(' ', ''):
        r = eq.SetEquationAndConfigurationOption(i, text, C('swAllConfiguration'), vnull())
        if r is False or (isinstance(r, int) and r < 0):
            raise RuntimeError(f'could not set equation {text}')
    if solve:
        eq.EvaluateAll()
    return i


# custom properties
def set_props(model, props, config=''):
    """props: dict name -> value (text). config '' = file-level"""
    cpm = model.Extension.CustomPropertyManager(config)
    for k, v in props.items():
        cpm.Add3(k, C('swCustomInfoText'), str(v), C('swCustomPropertyReplaceValue'))


def get_props(model, config=''):
    cpm = model.Extension.CustomPropertyManager(config)
    names = cpm.GetNames() or ()
    out = {}
    for n in names:
        out[n] = cpm.Get(n)
    return out


# transforms
def tf_from_sw(mt):
    """IMathTransform -> 4x4 column matrix (p_out = M @ p_in)"""
    a = np.array(mt.ArrayData, float)
    R = a[:9].reshape(3, 3)      # rows = images of the basis vectors (row-vector convention)
    M = np.eye(4)
    M[:3, :3] = R.T * a[12]
    M[:3, 3] = a[9:12]
    return M


def tf_to_sw(sw, M):
    R = M[:3, :3]
    s = np.cbrt(np.linalg.det(R))
    a = np.zeros(16)
    a[:9] = (R / s).T.reshape(9)
    a[9:12] = M[:3, 3]
    a[12] = s
    return sw.GetMathUtility().CreateTransform(vr8(a))


def cs_transform(model, cs_name):
    """4x4 of the named coordinate system as returned by the api"""
    mt = model.Extension.GetCoordinateSystemTransformByName(cs_name)
    if mt is None:
        raise RuntimeError(f'no coordinate system {cs_name!r}')
    return tf_from_sw(mt)


# mass properties
def massprops(model, sw=None, cs_name=None, items=None):
    """mass props of a part or assembly doc, SI.

    returns dict(m, cg, I_origin_sw, I_cg_sw, V) where I_* are the raw 3x3 arrays
    the api returns (solidworks sign convention, see stage-0 report). frame =
    the model frame, or cs_name if given (transform passed exactly as the api
    returns it; stage 0 proves which way round this has to be).
    """
    mp = model.Extension.CreateMassProperty2()
    if mp is None:
        raise RuntimeError('CreateMassProperty2 returned None (no solid bodies?)')
    mp.UseSystemUnits = True
    if items is not None:
        mp.SelectedItems = vdisp_arr(items)
    if cs_name:
        mt = model.Extension.GetCoordinateSystemTransformByName(cs_name)
        if not mp.SetCoordinateSystem(mt):
            raise RuntimeError(f'SetCoordinateSystem({cs_name}) failed')
    mp.Recalculate()
    return dict(m=float(mp.Mass), cg=np.array(mp.CenterOfMass, float), V=float(mp.Volume),
                I_origin=np.array(mp.GetMomentOfInertia(C('swMassPropertyMomentAboutCoordSys')), float).reshape(3, 3),
                I_cg=np.array(mp.GetMomentOfInertia(C('swMassPropertyMomentAboutCenterOfMass')), float).reshape(3, 3),
                density=float(mp.Density))


def override_mass(model, mass_kg=None, cg_m=None, cs_name='', config_opt='swThisConfiguration'):
    """document mass override via IMassProperty2 (the Override Mass Properties dialog)"""
    mp = model.Extension.CreateMassProperty2()
    opts = mp.GetOverrideOptions()
    if mass_kg is not None:
        opts.OverrideMass = True
        opts.SetOverrideMassValue(float(mass_kg))
    if cg_m is not None:
        opts.OverrideCenterOfMass = True
        opts.SetOverrideCenterOfMassValue(vr8(cg_m), cs_name)
    ok = mp.SetOverrideOptions(opts, C(config_opt), vnull())
    if not ok:
        raise RuntimeError('SetOverrideOptions failed')
    return ok


def clear_override(model, config_opt='swThisConfiguration'):
    mp = model.Extension.CreateMassProperty2()
    opts = mp.GetOverrideOptions()
    opts.OverrideMass = False
    opts.OverrideCenterOfMass = False
    opts.OverrideMomentsOfInertia = False
    return mp.SetOverrideOptions(opts, C(config_opt), vnull())


# numpy side: combine rigid bodies
def tensor_from_sw(I_sw):
    """solidworks moment array -> tensor notation (off-diagonals = -integral(xy dm)).
    stage 0 decides the sign; SW_PRODUCTS_ARE_POSITIVE_INTEGRALS is set from that test."""
    I = np.array(I_sw, float).copy()
    if SW_PRODUCTS_ARE_POSITIVE_INTEGRALS:
        off = ~np.eye(3, dtype=bool)
        I[off] = -I[off]
    return I


SW_PRODUCTS_ARE_POSITIVE_INTEGRALS = True   # confirmed by tools/stage0_api_sanity.py


def shift_to_point(m, cg, I_cg, p):
    """tensor about cg -> tensor about point p (same axes), tensor notation"""
    d = np.asarray(cg, float) - np.asarray(p, float)
    return I_cg + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def combine(bodies):
    """bodies: list of dict(m, cg (3,), I_cg (3x3 tensor)) in one frame -> same dict"""
    m = sum(b['m'] for b in bodies)
    if m <= 0:
        return dict(m=0.0, cg=np.zeros(3), I_cg=np.zeros((3, 3)))
    cg = sum(b['m'] * np.asarray(b['cg']) for b in bodies) / m
    I = np.zeros((3, 3))
    for b in bodies:
        I += shift_to_point(b['m'], b['cg'], b['I_cg'], cg)
    return dict(m=m, cg=cg, I_cg=I)


def transform_body(b, M):
    """re-express a body's mass props through a 4x4 rigid transform"""
    R = M[:3, :3]
    return dict(m=b['m'], cg=(M @ np.r_[b['cg'], 1.0])[:3], I_cg=R @ b['I_cg'] @ R.T)


def check_calls(calls):
    missing = verify(calls)
    if missing:
        raise RuntimeError(f'api members not in the installed typelib: {missing}')
