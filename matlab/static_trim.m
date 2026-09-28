function s = static_trim(P, cfg)
%STATIC_TRIM  Simplified longitudinal balance (the May-2026 report's trim).
%
%   s = static_trim(P, cfg)   cfg.V, cfg.gamma, cfg.mass, cfg.h, cfg.prop_model
%
% Solves   L = W cos(gamma)      (thrust tilt ignored)
%          Cm_aero = 0           (thrust-line moment ignored)
%          T = D + W sin(gamma)
% This is the balance behind Table 3 of the report. It is used as the
% regression reference and as the initial guess for the full trim in trim.m.
% The full trim (trim.m) differs from it by the thrust tilt (~0.1 deg alpha)
% and, for the unloaded aircraft, by the thrust-line moment (~0.3 deg de).

    if ~isfield(cfg, 'gamma'), cfg.gamma = 0; end
    if ~isfield(cfg, 'mass'),  cfg.mass = 'loaded'; end
    if ~isfield(cfg, 'h'),     cfg.h = P.trim_ref.altitude_m; end
    if ~isfield(cfg, 'prop_model'), cfg.prop_model = P.prop.model; end
    lon = P.aero.lon;
    rho = isa_density(cfg.h, P);
    W = P.mass.(cfg.mass).m * P.env.g;
    qS = 0.5 * rho * cfg.V^2 * P.geom.S;
    CL = W * cos(cfg.gamma) / qS;
    % linear 2x2: CLa*a + CLde*de = CL - CL0 ;  Cma*a + Cmde*de = -Cm0
    A = [lon.CL_alpha, lon.CL_de; lon.Cm_alpha, lon.Cm_de];
    z = A \ [CL - lon.CL0; -lon.Cm0];
    alpha = z(1); de = z(2);
    CD = P.aero.CD0 + P.aero.K * CL^2 + lon.CD_de*abs(de);
    T = qS * CD + W * sin(cfg.gamma);
    % invert the thrust model for dT (monotone in dT)
    dT = fzero(@(d) propulsion(cfg.V, d, P, cfg.prop_model) - T, [0, 1]);
    s = struct('alpha', alpha, 'de', de, 'dT', dT, 'T', T, 'CL', CL, 'CD', CD, ...
               'alpha_deg', rad2deg(alpha), 'de_deg', rad2deg(de), 'rho', rho, 'qS', qS);
end
