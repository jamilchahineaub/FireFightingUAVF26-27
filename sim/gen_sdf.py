#!/usr/bin/env python3
"""Generate the Gazebo SDF for prandtl_amph from params/params.json.

    python3 sim/gen_sdf.py [--config loaded|unloaded] [--out DIR]

FRD -> FLU: (x, y, z) -> (x, -y, -z). SDF <ixz> = +Ixz (aircraft convention).
Lateral coefficient signs in LATERAL_SIGN are unverified; set them from the
AdvancedLiftDrag vs aero.m parity test. Don't edit the generated SDF by hand.
"""
import argparse
import json
import pathlib

import jinja2
import yaml

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
PARAMS = ROOT / "params" / "params.json"

# sign factors applied to lateral coefficients when writing the plugin block
LATERAL_SIGN = {
    "CY_beta": +1, "Cl_beta": +1, "Cn_beta": +1,
    "CY_p": +1, "Cl_p": +1, "Cn_p": +1,
    "CY_r": +1, "Cl_r": +1, "Cn_r": +1,
    "CY_da": +1, "Cl_da": +1, "Cn_da": +1,
    "CY_dr": +1, "Cl_dr": +1, "Cn_dr": +1,
}


def v(d):
    """Flatten {value, source} entries."""
    return d["value"] if isinstance(d, dict) and "value" in d else d


def mass_props(raw, config):
    """Mirror of matlab/mass_props.m (mass-station model), in FRD."""
    mp = raw["mass_properties"]
    g = raw["geometry"]
    if mp["use_rounded_mass"]:
        m_af = mp["airframe"]["m_rounded"]
        m_pl = mp["loaded"]["m_rounded"] - m_af
    else:
        m_af = mp["airframe"]["m"]
        m_pl = mp["payload"]["m"]
    if mp.get("use_cad"):
        c = mp["cad"][config]
        return dict(m=c["m"], cg=c["cg_frd"], Ixx=c["Ixx"], Iyy=c["Iyy"], Izz=c["Izz"], Ixz=c["Ixz"])
    kx = mp["airframe"]["kx_over_b"] * g["b"]
    ky = mp["airframe"]["ky_over_Lfus"] * g["L_fus"]
    kz = (kx**2 + ky**2) ** 0.5
    Ixz = mp["Ixz_airframe"]
    I_af = [m_af * kx**2, m_af * ky**2, m_af * kz**2]
    if config == "unloaded":
        return dict(m=m_af, cg=[0.0, 0.0, 0.0], Ixx=I_af[0], Iyy=I_af[1], Izz=I_af[2], Ixz=Ixz)
    r_pl = mp["payload"]["r_from_airframe_cg_frd"]
    m_tot = m_af + m_pl
    cg = [m_pl * r / m_tot for r in r_pl]
    d_af = [-c for c in cg]
    d_pl = [r - c for r, c in zip(r_pl, cg)]

    def shift(d, m):
        dd = sum(x * x for x in d)
        return [m * (dd - d[0] ** 2), m * (dd - d[1] ** 2), m * (dd - d[2] ** 2)], m * d[0] * d[2]

    s_af, xz_af = shift(d_af, m_af)
    s_pl, xz_pl = shift(d_pl, m_pl)
    I = [I_af[i] + s_af[i] + s_pl[i] for i in range(3)]
    # aircraft-convention Ixz = integral(x z dm): point masses add m*x*z
    Ixz_tot = Ixz + xz_af + xz_pl
    return dict(m=m_tot, cg=cg, Ixx=I[0], Iyy=I[1], Izz=I[2], Ixz=Ixz_tot)


def frd_to_flu(p):
    return [p[0], -p[1], -p[2]]


def build_context(raw, config):
    geom = raw["geometry"]
    lon = {k: v(x) for k, x in raw["aero"]["longitudinal"].items()}
    lat = {k: v(x) * LATERAL_SIGN.get(k, 1) for k, x in raw["aero"]["lateral"].items()}
    mp = mass_props(raw, config)
    # blend centre calibrated like load_params.m (peak of blended curve = CL_max)
    alpha_stall = calibrate_stall(lon["CL0"], lon["CL_alpha"], raw["aero"]["stall"]["CL_max"],
                                  raw["aero"]["stall"]["blend_M"])
    thrust_line = raw["propulsion"]["thrust_line"]["r_thrust_from_loaded_cg_frd"]
    # thrust point relative to THIS configuration's CG (FRD), then FLU
    cg_loaded = mass_props(raw, "loaded")["cg"]
    thrust_pt_frd = [t + cl - c for t, cl, c in zip(thrust_line, cg_loaded, mp["cg"])]
    return dict(
        meta=raw["meta"], config=config, sha=raw["meta"]["sha256"],
        geom=geom, rho=raw["environment"]["rho_sl"],
        mass=mp["m"], I=mp, cg_flu=frd_to_flu(mp["cg"]),
        ixz_sdf=mp["Ixz"],               # SDF <ixz> = +Ixz_aircraft (see module docstring)
        lon=lon, lat=lat, CD0=raw["aero"]["drag_polar"]["CD0"], K=raw["aero"]["drag_polar"]["K"],
        alpha_stall=alpha_stall, blend_M=raw["aero"]["stall"]["blend_M"],
        thrust_pt_flu=frd_to_flu(thrust_pt_frd),
        n_motors=raw["propulsion"]["n_motors"],
        act=raw["actuators"], mix=raw["aero"]["mixing"]["Mix"],
        hull_boxes=raw["hydro"].get("hull_boxes", []),
    )


def calibrate_stall(CL0, CLa, CLmax, M):
    import math
    al = [i * math.radians(40) / 2000 for i in range(2001)]

    def peak(a_s):
        best = -1e9
        for a in al:
            num = 1 + math.exp(-M * (a - a_s)) + math.exp(M * (a + a_s))
            den = (1 + math.exp(-M * (a - a_s))) * (1 + math.exp(M * (a + a_s)))
            sg = num / den
            CL = (1 - sg) * (CL0 + CLa * a) + sg * (2 * math.copysign(1, a) * math.sin(a) ** 2 * math.cos(a) if a else 0)
            best = max(best, CL)
        return best

    lo = (CLmax - CL0) / CLa
    hi = lo + math.radians(15)
    for _ in range(60):  # bisection on peak(a_s) - CLmax (monotone increasing)
        mid = 0.5 * (lo + hi)
        if peak(mid) - CLmax < 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="loaded", choices=["loaded", "unloaded"])
    ap.add_argument("--out", default=str(HERE / "generated"))
    ap.add_argument("--params", default=str(PARAMS))
    args = ap.parse_args()

    raw = json.loads(pathlib.Path(args.params).read_text())
    ctx = build_context(raw, args.config)
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(str(HERE / "templates")),
                             undefined=jinja2.StrictUndefined, trim_blocks=True, lstrip_blocks=True)
    sdf = env.get_template("model.sdf.j2").render(**ctx)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    dst = out / f"prandtl_amph_{args.config}.sdf"
    dst.write_text(sdf)
    # sidecar with the exact numbers used (for the MATLAB parse-back test)
    (out / f"prandtl_amph_{args.config}.json").write_text(json.dumps(
        dict(sha=ctx["sha"], mass=ctx["mass"], I=ctx["I"], cg_flu=ctx["cg_flu"],
             alpha_stall=ctx["alpha_stall"], lon=ctx["lon"], lat=ctx["lat"]), indent=1))
    print(f"wrote {dst}  (m={ctx['mass']:.3f} kg, Iyy={ctx['I']['Iyy']:.4f}, sha {ctx['sha'][:12]})")


if __name__ == "__main__":
    main()
