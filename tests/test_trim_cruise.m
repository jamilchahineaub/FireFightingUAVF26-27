function [pass, msg] = test_trim_cruise()
%TEST_TRIM_CRUISE  Regression against the May-2026 report, Table 3 (two parts).
%
%  Part A  static_trim.m (L = W, Cm_aero = 0, T = D) must reproduce Table 3:
%          loaded   alpha 6.33  de -1.83  dT 0.279
%          unloaded alpha 4.46  de -0.59  dT 0.244
%          This proves the coefficient set in params.yaml is the report's.
%
%  Part B  the full nonlinear trim.m must converge with zero lateral unknowns,
%          and its deviation from Table 3 must be the physics the static
%          balance leaves out: thrust tilt (alpha) and, for the unloaded
%          aircraft, the thrust-line moment. The elevator shift is predicted
%          analytically and compared.
    P = load_params();
    tolA = P.trim_ref.tolerance;
    tolB = P.trim_ref.full_trim_tolerance;
    msgs = {}; ok = true;
    for f = {'cruise_loaded', 'cruise_unloaded'}
        ref = P.trim_ref.(f{1});
        cfg = struct('V', ref.V, 'gamma', ref.gamma, 'mass', ref.mass, ...
                     'prop_model', 'legacy_constant_power');

        % ---- A: static balance ----------------------------------------------
        s = static_trim(P, cfg);
        okA = abs(s.alpha_deg - ref.alpha_deg) < tolA.alpha_deg && ...
              abs(s.de_deg - ref.de_deg) < tolA.de_deg && ...
              abs(s.dT - ref.dT) < tolA.dT;

        % ---- B: full trim ---------------------------------------------------
        [~, ~, info] = trim(P, cfg);
        lat = max(abs([info.beta info.da info.dr info.phi]));
        okB = abs(info.alpha_deg - ref.alpha_deg) < tolB.alpha_deg && ...
              abs(info.de_deg - ref.de_deg) < tolB.de_deg && ...
              abs(info.dT - ref.dT) < tolB.dT && lat < 1e-8 && ...
              info.exitflag > 0 && info.resnorm < 1e-8;

        % predicted elevator shift: thrust-line moment + alpha shift from thrust tilt
        %   Cm_aero + Cm_thrust = 0  ->  Cm_de*d(de) = -Cm_alpha*d(alpha) - T*rz/(qS c)
        rz = P.mass.(ref.mass).r_thrust(3);
        dalpha = info.alpha - s.alpha;
        dde_pred = (-P.aero.lon.Cm_alpha * dalpha - info.T * rz / (s.qS * P.geom.c_bar)) ...
                   / P.aero.lon.Cm_de;                                          % rad
        dde_act  = info.de - s.de;
        okC = abs(dde_act - dde_pred) < deg2rad(0.03);

        ok = ok && okA && okB && okC;
        msgs{end+1} = sprintf(['%s: static a=%.2f de=%.2f dT=%.4f [ref %.2f/%.2f/%.3f] | ' ...
            'full a=%.2f de=%.2f dT=%.4f, d(de) %.2f deg (thrust-moment pred %.2f)'], ...
            f{1}, s.alpha_deg, s.de_deg, s.dT, ref.alpha_deg, ref.de_deg, ref.dT, ...
            info.alpha_deg, info.de_deg, info.dT, rad2deg(dde_act), rad2deg(dde_pred));
    end
    pass = ok;
    msg = strjoin(msgs, ' || ');
end
