function [xdot, y] = eom(t, x, u, P, cfg)
%EOM  Nonlinear rigid-body 6-DOF equations of motion, body axes (FRD), NED inertial.
%
%   [xdot, y] = eom(t, x, u, P, cfg)
%
% State x (18x1):
%   1:3   pN pE pD        position, NED [m]           (h = -pD)
%   4:6   u  v  w         body velocity, FRD [m/s]
%   7:10  q0 q1 q2 q3     quaternion NED->body, scalar first
%   11:13 p  q  r         body rates [rad/s]
%   14:18 de da dr dT dL  actuator states [rad rad rad - rad]
%
% Input u (5x1): commanded [de da dr dT dL]
%
% cfg fields (all optional):
%   .mass     'loaded' (default) | 'unloaded'  -> selects P.mass.<cfg>
%   .wind_ned steady wind vector in NED [m/s], default [0 0 0]
%   .gust_b   body-axis gust velocity [m/s] (function of t or vector), default 0
%   .hydro    function handle with hydro_stub signature, used if P.hydro.enabled
%   .prop_model override for propulsion.m
%   .aero_off / .grav_off / .thrust_off  logical switches for unit tests
%
% y: diagnostics (Va, alpha, beta, forces by source, Euler angles, coefficients)
%
% Equations (Beard & McLain ch. 3, Stevens & Lewis ch. 1):
%   pdot   = R_b2n v_b
%   vdot_b = F/m + R_n2b [0 0 g]' - omega x v_b
%   qdot   = 1/2 Omega(omega) q  (+ norm correction)
%   wdot   = J^-1 (M - omega x J omega),  J full tensor incl. Ixz

    if nargin < 5, cfg = struct(); end
    if ~isfield(cfg, 'mass'), cfg.mass = 'loaded'; end
    if ~isfield(cfg, 'wind_ned'), cfg.wind_ned = [0; 0; 0]; end
    if ~isfield(cfg, 'aero_off'), cfg.aero_off = false; end
    if ~isfield(cfg, 'grav_off'), cfg.grav_off = false; end
    if ~isfield(cfg, 'thrust_off'), cfg.thrust_off = false; end
    if ~isfield(cfg, 'prop_model'), cfg.prop_model = P.prop.model; end

    Q = quat_utils();
    mp = P.mass.(cfg.mass);
    m = mp.m; J = mp.J; g = P.env.g;

    pos  = x(1:3);
    vb   = x(4:6);
    quat = x(7:10);
    om   = x(11:13);
    dact = x(14:18);
    quat = quat / norm(quat);

    R_b2n = Q.R_b2n(quat);
    R_n2b = R_b2n';

    % ---- relative wind ------------------------------------------------------
    wind_b = R_n2b * cfg.wind_ned(:);
    if isfield(cfg, 'gust_b')
        if isa(cfg.gust_b, 'function_handle'), wind_b = wind_b + cfg.gust_b(t);
        else, wind_b = wind_b + cfg.gust_b(:); end
    end
    vr = vb - wind_b;
    Va = norm(vr);
    if Va > 1e-6
        alpha = atan2(vr(3), vr(1));
        beta  = asin(max(-1, min(1, vr(2) / Va)));
    else
        alpha = 0; beta = 0;
    end
    h = -pos(3);
    rho = isa_density(h, P);

    % ---- forces and moments -------------------------------------------------
    d = struct('de', dact(1), 'da', dact(2), 'dr', dact(3), 'dL', dact(5));
    if cfg.aero_off
        Fa = zeros(3,1); Ma = zeros(3,1); ya = struct('CL',0,'CD',0,'Cm',0,'stall',false);
    else
        [Fa, Ma, ya] = aero(Va, alpha, beta, om, d, rho, P);
    end

    if cfg.thrust_off
        T = 0; Ft = zeros(3,1); Mt = zeros(3,1);
    else
        T = propulsion(Va, dact(4), P, cfg.prop_model);
        Ft = [T; 0; 0];
        Mt = cross(mp.r_thrust(:), Ft);          % thrust below CG (+z) -> nose-up (+My)
    end

    if cfg.grav_off
        Fg = zeros(3,1);
    else
        Fg = R_n2b * [0; 0; m * g];
    end

    Fh = zeros(3,1); Mh = zeros(3,1);
    if isfield(P, 'hydro') && P.hydro.enabled && isfield(cfg, 'hydro')
        [Fh, Mh] = cfg.hydro(pos, vb, quat, om, P);
    end

    F = Fa + Ft + Fg + Fh;
    M = Ma + Mt + Mh;

    % ---- kinematics and dynamics --------------------------------------------
    pdot = R_b2n * vb;
    vdot = F / m - cross(om, vb);
    qdot = Q.deriv(quat, om);
    wdot = J \ (M - cross(om, J * om));
    ddot = actuator(dact, u(:), P);

    xdot = [pdot; vdot; qdot; wdot; ddot];

    if nargout > 1
        [phi, theta, psi] = Q.to_euler(quat);
        y = struct('Va', Va, 'alpha', alpha, 'beta', beta, 'h', h, 'rho', rho, ...
                   'phi', phi, 'theta', theta, 'psi', psi, ...
                   'F_aero', Fa, 'M_aero', Ma, 'F_thrust', Ft, 'M_thrust', Mt, ...
                   'F_grav', Fg, 'F_hydro', Fh, 'T', T, 'aero', ya, ...
                   'gamma', theta - alpha);
    end
end
