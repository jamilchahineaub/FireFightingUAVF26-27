function [xt, ut, info] = trim(P, cfg)
%TRIM  Steady straight flight trim: solve f(x*, u*) = 0 for given V and gamma.
%
%   [xt, ut, info] = trim(P, cfg)
%
%   cfg.V        airspeed [m/s]
%   cfg.gamma    flight-path angle [rad] (0 level, <0 descent)   default 0
%   cfg.mass     'loaded' | 'unloaded'                           default 'loaded'
%   cfg.h        altitude [m]                                    default P.trim_ref.altitude_m
%   cfg.prop_model  override propulsion model                    default P.prop.model
%   cfg.z0       optional initial guess [alpha de dT beta da dr phi]
%
% Unknowns z = [alpha, de, dT, beta, da, dr, phi]; theta = alpha + gamma; psi = 0.
% Residuals: udot, vdot, wdot, pdot, qdot, rdot (6 force/moment equations) and
% phi (wings level). 7 unknowns, 7 residuals, solved with fsolve.
%
% Returns the full eom state xt (18x1), the trim input ut (5x1) and info with
% alpha, theta, de, dT, T, CL, exitflag, residual norm.

    if ~isfield(cfg, 'gamma'), cfg.gamma = 0; end
    if ~isfield(cfg, 'mass'),  cfg.mass = 'loaded'; end
    if ~isfield(cfg, 'h'),     cfg.h = P.trim_ref.altitude_m; end
    if ~isfield(cfg, 'prop_model'), cfg.prop_model = P.prop.model; end

    V = cfg.V; gam = cfg.gamma;
    rho = isa_density(cfg.h, P);

    % ---- initial guess from the static equations ----------------------------
    if isfield(cfg, 'z0')
        z0 = cfg.z0(:);
    else
        s = static_trim(P, cfg);
        z0 = [s.alpha; s.de; s.dT; 0; 0; 0; 0];
    end

    ecfg = struct('mass', cfg.mass, 'prop_model', cfg.prop_model);
    res = @(z) residual(z, V, gam, cfg.h, P, ecfg);

    opts = optimset('Display', 'off', 'TolFun', 1e-14, 'TolX', 1e-12, 'MaxIter', 400, ...
                    'MaxFunEvals', 4000);
    [z, fval, exitflag] = fsolve(res, z0, opts);

    [xt, ut] = build_state(z, V, gam, cfg.h);
    [~, y] = eom(0, xt, ut, P, ecfg);

    info = struct('alpha', z(1), 'theta', z(1) + gam, 'de', z(2), 'dT', z(3), ...
                  'beta', z(4), 'da', z(5), 'dr', z(6), 'phi', z(7), ...
                  'V', V, 'gamma', gam, 'mass', cfg.mass, 'h', cfg.h, 'rho', rho, ...
                  'T', y.T, 'CL', y.aero.CL, 'CD', y.aero.CD, ...
                  'exitflag', exitflag, 'resnorm', norm(fval), 'z', z, ...
                  'prop_model', cfg.prop_model);
    info.alpha_deg = rad2deg(z(1)); info.de_deg = rad2deg(z(2));
    if exitflag <= 0 || norm(fval) > 1e-6
        warning('trim:noconverge', 'trim did not converge (exitflag %d, |res| %.2e)', exitflag, norm(fval));
    end
end

function r = residual(z, V, gam, h, P, ecfg)
    [x, u] = build_state(z, V, gam, h);
    xd = eom(0, x, u, P, ecfg);
    r = [xd(4:6); xd(11:13); z(7)];          % udot vdot wdot pdot qdot rdot, phi=0
end

function [x, u] = build_state(z, V, gam, h)
    alpha = z(1); de = z(2); dT = z(3); beta = z(4); da = z(5); dr = z(6); phi = z(7);
    theta = alpha + gam;
    vb = V * [cos(alpha)*cos(beta); sin(beta); sin(alpha)*cos(beta)];
    act = [de da dr dT 0];
    x = make_state('pos', [0 0 -h], 'vb', vb', 'euler', [phi theta 0], 'pqr', [0 0 0], 'act', act);
    u = act(:);
end
