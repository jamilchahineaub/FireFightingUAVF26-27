function [pass, msg] = test_sdf_matches_matlab()
%TEST_SDF_MATCHES_MATLAB  The SDF generator's mass/inertia/stall must equal MATLAB's.
%  Reads sim/generated/prandtl_amph_<cfg>.json (sidecar written by gen_sdf.py)
%  and compares with load_params(). Skips with a warning if the sidecar is
%  missing (run `make sdf` first).
    P = load_params();
    here = fileparts(mfilename('fullpath'));
    ok = true; msgs = {};
    for cfg = {'loaded', 'unloaded'}
        f = fullfile(here, '..', 'sim', 'generated', ['prandtl_amph_' cfg{1} '.json']);
        if ~exist(f, 'file')
            pass = false; msg = 'sidecar missing: run `make sdf` (python3 sim/gen_sdf.py --config loaded/unloaded)';
            return;
        end
        S = jsondecode(fileread(f));
        Mm = P.mass.(cfg{1});
        e = [abs(S.mass - Mm.m), abs(S.I.Ixx - Mm.J(1,1)), abs(S.I.Iyy - Mm.J(2,2)), ...
             abs(S.I.Izz - Mm.J(3,3)), abs(S.I.Ixz + Mm.J(1,3)), ...
             abs(S.cg_flu(3) + Mm.cg(3)), abs(S.alpha_stall - P.aero.alpha_stall)];
        oki = all(e < 1e-4) && strcmp(S.sha, P.meta.sha256);
        ok = ok && oki;
        msgs{end+1} = sprintf('%s max|diff|=%.1e sha %s', cfg{1}, max(e), ternary(strcmp(S.sha, P.meta.sha256), 'match', 'MISMATCH'));
    end
    pass = ok; msg = strjoin(msgs, '; ');
end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end
