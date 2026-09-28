function [T, info] = propulsion(Va, dT, P, model)
%PROPULSION  Total thrust of both motors as a function of airspeed and throttle.
%
%   [T, info] = propulsion(Va, dT, P)          uses P.prop.model
%   [T, info] = propulsion(Va, dT, P, model)   model = 'legacy_constant_power' | 'momentum'
%
% legacy_constant_power: T = P_prop_max*dT/max(Va, V_min). Reproduces the
%   report trims; not consistent with installed power (D-02). Regression only.
% momentum: T = a*dT^2 - b*dT*Va, a = static thrust, b from the cruise anchor.
%
% info: struct with dT_dV (dT/dVa), dT_ddT (dT/d dT), P_prop (T*V), P_shaft.

    if nargin < 4, model = P.prop.model; end
    dT = min(max(dT, 0), 1);

    switch model
        case 'legacy_constant_power'
            m = P.prop.legacy_constant_power;
            V = max(Va, m.V_min);
            T = m.P_prop_max * dT / V;
            if Va > m.V_min, dTdV = -m.P_prop_max * dT / V^2; else, dTdV = 0; end
            dTddT = m.P_prop_max / V;
        case 'momentum'
            m = P.prop.momentum;
            a = m.T_static_total; b = m.b;
            T = max(a * dT^2 - b * dT * Va, 0);
            dTdV  = -b * dT;
            dTddT = 2 * a * dT - b * Va;
        otherwise
            error('propulsion:model', 'unknown propulsion model "%s"', model);
    end

    info.model   = model;
    info.dT_dV   = dTdV;
    info.dT_ddT  = dTddT;
    info.P_prop  = T * Va;
    info.P_shaft = T * Va / P.prop.eta_p;
    info.P_limit = P.prop.P_elec_max_total;
    info.over_power = info.P_shaft > info.P_limit;
end
