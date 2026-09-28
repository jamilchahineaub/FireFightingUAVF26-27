"""Parse-back test: the generated SDF must carry the YAML's mass, CG and inertia.

    python3 -m pytest -q sim/      (or just python3 sim/test_gen_sdf.py)
"""
import json
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent


def gen(config):
    subprocess.run([sys.executable, str(HERE / "gen_sdf.py"), "--config", config], check=True,
                   capture_output=True)
    sdf = ET.parse(HERE / "generated" / f"prandtl_amph_{config}.sdf")
    side = json.loads((HERE / "generated" / f"prandtl_amph_{config}.json").read_text())
    return sdf, side


def test_parse_back_loaded():
    check("loaded")


def test_parse_back_unloaded():
    check("unloaded")


def check(config):
    raw = json.loads((ROOT / "params" / "params.json").read_text())
    sdf, side = gen(config)
    inertial = sdf.find(".//link[@name='base_link']/inertial")
    m = float(inertial.find("mass").text)
    I = inertial.find("inertia")
    ixx, iyy, izz, ixz = (float(I.find(k).text) for k in ("ixx", "iyy", "izz", "ixz"))
    pose = [float(x) for x in inertial.find("pose").text.split()[:3]]

    exp_m = raw["mass_properties"][config]["m_rounded"]
    assert abs(m - exp_m) < 1e-6, (m, exp_m)
    assert abs(iyy - side["I"]["Iyy"]) < 1e-5
    assert abs(ixx - side["I"]["Ixx"]) < 1e-5
    assert abs(izz - side["I"]["Izz"]) < 1e-5
    assert abs(ixz - side["I"]["Ixz"]) < 1e-5
    # FRD -> FLU: z flips sign; loaded CG is 0.0343 m BELOW airframe CG -> FLU z = -0.0343
    if config == "loaded":
        assert abs(pose[2] + 4.0 * 0.12 / 14.0) < 1e-5, pose
        # report: Iyy drops by ~0.04 when the tank empties
        _, side_u = gen("unloaded")
        assert 0.035 < side["I"]["Iyy"] - side_u["I"]["Iyy"] < 0.045
    # sha recorded in header
    text = (HERE / "generated" / f"prandtl_amph_{config}.sdf").read_text()
    assert raw["meta"]["sha256"] in text
    # aero coefficients present
    plugin = sdf.find(".//plugin[@name='gz::sim::systems::AdvancedLiftDrag']")
    assert abs(float(plugin.find("Cema").text) - raw["aero"]["longitudinal"]["Cm_alpha"]["value"]) < 1e-12
    assert abs(float(plugin.find("Cemq").text) - raw["aero"]["longitudinal"]["Cm_q"]["value"]) < 1e-12


if __name__ == "__main__":
    test_parse_back_loaded()
    test_parse_back_unloaded()
    print("sim/test_gen_sdf.py: PASS")
