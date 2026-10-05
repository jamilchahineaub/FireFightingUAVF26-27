"""run the whole cl415 pipeline in order (windows side)

    .venv/Scripts/python build_all.py           everything, solidworks must be open
    .venv/Scripts/python build_all.py --no-sw   skip solidworks, reuse cad/solidworks/sw_mass_props.json
    .venv/Scripts/python build_all.py --wsl     also run the sdf export and the gazebo checks in the AMR distro

the gazebo side can also be run by hand inside WSL:
    bash tools/export_sdf.sh && bash verify/run_sim_tests.sh
"""
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
STEPS = [
    ('step 1  inspect the obj', ['obj_inspect/inspect_obj.py']),
    ('step 2  cadquery parts', ['cad/build_cad.py']),
    ('step 2  interference check', ['cad/check_interference.py']),
    ('step 3  mass budget and battery position', ['mass/mass_props.py', 'plan']),
    ('step 2b solidworks parts, assembly, mass properties', ['cad/solidworks/build_sw.py']),
    ('step 3  link inertials from solidworks', ['mass/mass_props.py', 'final']),
    ('step 4  aero parameters', ['aero/aero_params.py']),
    ('step 4  generated xacro', ['tools/export_description.py']),
]
WSL_DISTRO = 'AMR'


def wsl(script):
    path = '/mnt/' + ROOT[0].lower() + ROOT[2:].replace('\\', '/') + '/' + script
    env = dict(os.environ, MSYS_NO_PATHCONV='1')
    return subprocess.run(['wsl.exe', '-d', WSL_DISTRO, '--', 'bash', '-l', path], env=env).returncode


def main():
    t0 = time.time()
    for name, cmd in STEPS:
        if '--no-sw' in sys.argv and 'solidworks' in cmd[0]:
            print(f'-- {name}: skipped')
            continue
        print(f'-- {name}', flush=True)
        r = subprocess.run([PY, '-u'] + cmd, cwd=ROOT)
        if r.returncode:
            sys.exit(f'{cmd[0]} failed ({r.returncode})')
    if '--wsl' in sys.argv:
        for s in ('tools/export_sdf.sh', 'verify/run_sim_tests.sh'):
            print(f'-- wsl {s}', flush=True)
            if wsl(s):
                sys.exit(f'{s} failed')
        subprocess.run([PY, 'verify/analyse.py'], cwd=ROOT, check=True)
    print(f'done in {(time.time() - t0) / 60:.1f} min')


if __name__ == '__main__':
    main()
