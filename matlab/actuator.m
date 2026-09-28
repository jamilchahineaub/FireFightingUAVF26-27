function [ddot, d_sat] = actuator(d, cmd, P)
%ACTUATOR  First-order servo dynamics with deflection and rate limits.
%
%   [ddot, d_sat] = actuator(d, cmd, P)
%
%   d    current actuator states [de da dr dT dL]  (rad, rad, rad, 0..1, rad)
%   cmd  commanded values, same order
%   ddot state derivatives
%   d_sat the saturated command actually tracked
%
% Each channel:  ddot = sat_rate( (sat_pos(cmd) - d) / tau ).
% Throttle saturates to [min, max] and has no rate limit.

    A = P.act;
    ch = {A.elevator, A.aileron, A.rudder, [], A.direct_lift};
    ddot  = zeros(5, 1);
    d_sat = zeros(5, 1);
    for k = [1 2 3 5]
        a = ch{k};
        c = max(-a.delta_max, min(a.delta_max, cmd(k)));
        d_sat(k) = c;
        rate = (c - d(k)) / a.tau;
        ddot(k) = max(-a.rate_max, min(a.rate_max, rate));
    end
    c = max(A.throttle.min, min(A.throttle.max, cmd(4)));
    d_sat(4) = c;
    ddot(4) = (c - d(4)) / A.throttle.tau;
end
