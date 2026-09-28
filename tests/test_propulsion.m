function [pass, msg] = test_propulsion()
%TEST_PROPULSION  Both thrust models.
%  legacy : reproduces T = 16.74 N at (20 m/s, 0.279) and 14.61 N at (20, 0.244)
%  momentum: T(0,1) = T_static, T(V_anchor, dT_anchor) = T_anchor, dT/ddT > 0,
%            dT/dV <= 0, T >= 0, shaft power at cruise below the installed limit.
    P = load_params();
    msgs = {};
    tr = P.trim_ref;

    T1 = propulsion(tr.cruise_loaded.V, tr.cruise_loaded.dT, P, 'legacy_constant_power');
    T2 = propulsion(tr.cruise_unloaded.V, tr.cruise_unloaded.dT, P, 'legacy_constant_power');
    e1 = abs(T1 - tr.cruise_loaded.thrust_N) / tr.cruise_loaded.thrust_N;
    e2 = abs(T2 - tr.cruise_unloaded.thrust_N) / tr.cruise_unloaded.thrust_N;
    ok_leg = e1 < 0.01 && e2 < 0.01;
    msgs{end+1} = sprintf('legacy T=%.2f/%.2f N (ref %.2f/%.2f)', T1, T2, ...
        tr.cruise_loaded.thrust_N, tr.cruise_unloaded.thrust_N);

    m = P.prop.momentum;
    [Ts, ~] = propulsion(0, 1, P, 'momentum');
    [Ta, ia] = propulsion(m.V_anchor, m.dT_anchor, P, 'momentum');
    ok_s = abs(Ts - m.T_static_total) / m.T_static_total < 0.01;
    ok_a = abs(Ta - m.T_anchor) / m.T_anchor < 0.01;
    ok_d = ia.dT_ddT > 0 && ia.dT_dV <= 0;
    ok_p = ia.P_shaft <= m.P_check_W;
    Tneg = propulsion(60, 0.05, P, 'momentum');
    ok_n = Tneg >= 0;
    msgs{end+1} = sprintf('momentum T(0,1)=%.1f T(%.0f,%.2f)=%.1f Pshaft=%.0fW', ...
        Ts, m.V_anchor, m.dT_anchor, Ta, ia.P_shaft);

    % document the inconsistency of the legacy model (informational, not a failure)
    [~, il] = propulsion(20, 1.0, P, 'legacy_constant_power');
    msgs{end+1} = sprintf('NOTE legacy full-throttle shaft power at 20 m/s = %.0f W (installed %.0f W)', ...
        il.P_shaft, P.prop.P_elec_max_total);

    pass = ok_leg && ok_s && ok_a && ok_d && ok_p && ok_n;
    msg = strjoin(msgs, '; ');
end
