"""read the installed solidworks type libraries (sldworks.tlb, swconst.tlb)

enum values and interface member names come from the installed version,
never from memory. dumps a json cache in tools/_tlb/ and offers
    C('swDocPART')                 -> enum value
    has('IMassProperty2', 'Mass')  -> bool
    verify(calls)                  -> list of missing (iface, member)
"""
import json
import os
import winreg

import pythoncom

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, '_tlb')


def sw_install_dir():
    # COM server path is the install that Dispatch will actually start
    k = winreg.OpenKey(winreg.HKEY_CLASSES_ROOT, r'SldWorks.Application\CLSID')
    clsid = winreg.QueryValue(k, '')
    k2 = winreg.OpenKey(winreg.HKEY_CLASSES_ROOT, rf'CLSID\{clsid}\LocalServer32')
    exe = winreg.QueryValue(k2, '').strip('"').split(' /')[0]
    import win32api
    return os.path.dirname(win32api.GetLongPathName(exe))


def _dump_enums(tlb):
    out = {}
    for i in range(tlb.GetTypeInfoCount()):
        if tlb.GetTypeInfoType(i) != pythoncom.TKIND_ENUM:
            continue
        ti = tlb.GetTypeInfo(i)
        name = tlb.GetDocumentation(i)[0]
        ta = ti.GetTypeAttr()
        vals = {}
        for j in range(ta.cVars):
            vd = ti.GetVarDesc(j)
            vals[ti.GetNames(vd.memid)[0]] = vd.value
        out[name] = vals
    return out


def _dump_ifaces(tlb):
    out = {}
    for i in range(tlb.GetTypeInfoCount()):
        kind = tlb.GetTypeInfoType(i)
        if kind not in (pythoncom.TKIND_INTERFACE, pythoncom.TKIND_DISPATCH):
            continue
        ti = tlb.GetTypeInfo(i)
        name = tlb.GetDocumentation(i)[0]
        ta = ti.GetTypeAttr()
        mem = {}
        for j in range(ta.cFuncs):
            fd = ti.GetFuncDesc(j)
            names = ti.GetNames(fd.memid)
            kindmap = {1: 'method', 2: 'get', 4: 'put', 8: 'putref'}
            mem.setdefault(names[0], []).append(dict(kind=kindmap.get(fd.invkind, fd.invkind),
                                                     args=list(names[1:]), nparams=len(fd.args)))
        for j in range(ta.cVars):
            vd = ti.GetVarDesc(j)
            mem.setdefault(ti.GetNames(vd.memid)[0], []).append(dict(kind='var'))
        out.setdefault(name, {}).update(mem)
    return out


def build_cache(force=False):
    os.makedirs(CACHE, exist_ok=True)
    fe = os.path.join(CACHE, 'swconst_enums.json')
    fi = os.path.join(CACHE, 'sldworks_members.json')
    if not force and os.path.exists(fe) and os.path.exists(fi):
        return
    d = sw_install_dir()
    enums = _dump_enums(pythoncom.LoadTypeLib(os.path.join(d, 'swconst.tlb')))
    ifaces = _dump_ifaces(pythoncom.LoadTypeLib(os.path.join(d, 'sldworks.tlb')))
    json.dump(enums, open(fe, 'w'), indent=0, sort_keys=True)
    json.dump(ifaces, open(fi, 'w'), indent=0, sort_keys=True)
    json.dump(dict(install_dir=d), open(os.path.join(CACHE, 'source.json'), 'w'))


build_cache()
ENUMS = json.load(open(os.path.join(CACHE, 'swconst_enums.json')))
IFACES = json.load(open(os.path.join(CACHE, 'sldworks_members.json')))
_FLAT = {}
for _e, _vals in ENUMS.items():
    for _m, _v in _vals.items():
        _FLAT.setdefault(_m, (_e, _v))


def C(member, enum=None):
    """enum value by member name (optionally qualified by enum type)"""
    if enum:
        return ENUMS[enum][member]
    return _FLAT[member][1]


def has(iface, member):
    for name in (iface, iface.lstrip('I'), 'I' + iface):
        if name in IFACES and member in IFACES[name]:
            return True
    return False


def verify(calls):
    return [(i, m) for i, m in calls if not has(i, m)]


if __name__ == '__main__':
    import sys
    build_cache(force='--force' in sys.argv)
    print('enums', len(ENUMS), 'interfaces', len(IFACES))
