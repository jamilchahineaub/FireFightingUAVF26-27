function [F, M, y] = aero(Va, alpha, beta, pqr, d, rho, P)
%AERO  Aerodynamic force and moment build-up in stability-derivative form.
%
%   [F, M, y] = aero(Va, alpha, beta, pqr, d, rho, P)
%
%   Va     airspeed [m/s]
%   alpha  angle of attack [rad], beta sideslip [rad]
%   pqr    body rates [p q r] [rad/s]
%   d      surface deflections struct: d.de d.da d.dr d.dL [rad]
%          (autopilot commands; the front/rear mixing is applied downstream in
%           the SDF/PX4 allocation. With d.dL = 0 this is a conventional model.)
%   rho    air density [kg/m^3]
%   P      params struct from load_params
%
%   F, M   body-axis (FRD) force [N] and moment [N m] about the CG
%   y      diagnostics: CL, CD, CY, Cl, Cm, Cn, L, D, q_bar, stall flag, sigma
%
% Lift uses the Beard & McLain sigmoid blend between the linear model and a
% flat-plate post-stall model. Same structure as Gazebo AdvancedLiftDrag
% (alphaStall, M), so the MATLAB and SDF aero can be compared one-to-one.
% Drag is the sizing-tool parabolic polar. Moments use the handbook/AVL
% derivatives. All derivatives per radian.

    lon = P.aero.lon; lat = P.aero.lat;
    b = P.geom.b; c = P.geom.c_bar; S = P.geom.S;
    p = pqr(1); q = pqr(2); r = pqr(3);

    Va = max(Va, 1e-3);
    qbar = 0.5 * rho * Va^2;
    phat = p * b / (2 * Va);
    qhat = q * c / (2 * Va);
    rhat = r * b / (2 * Va);

    % ---- lift: linear / post-stall blend ------------------------------------
    a_s = P.aero.alpha_stall;  Mb = P.aero.blend_M;
    num = 1 + exp(-Mb*(alpha - a_s)) + exp(Mb*(alpha + a_s));
    den = (1 + exp(-Mb*(alpha - a_s))) * (1 + exp(Mb*(alpha + a_s)));
    sigma = num / den;
    CL_lin  = lon.CL0 + lon.CL_alpha * alpha;
    CL_flat = 2 * sign(alpha) * sin(alpha)^2 * cos(alpha);
    CL_a = (1 - sigma) * CL_lin + sigma * CL_flat;
    CL = CL_a + lon.CL_q * qhat + lon.CL_de * d.de + lon.CL_dL * d.dL;

    % ---- drag: parabolic polar on the total lift coefficient ----------------
    CD = P.aero.CD0 + P.aero.K * CL^2 + lon.CD_de * abs(d.de);

    % ---- pitching moment ----------------------------------------------------
    Cm = lon.Cm0 + lon.Cm_alpha * alpha + lon.Cm_q * qhat ...
         + lon.Cm_de * d.de + lon.Cm_dL * d.dL;

    % ---- lateral-directional ------------------------------------------------
    CY = lat.CY0 + lat.CY_beta*beta + lat.CY_p*phat + lat.CY_r*rhat ...
         + lat.CY_da*d.da + lat.CY_dr*d.dr;
    Cl = lat.Cl0 + lat.Cl_beta*beta + lat.Cl_p*phat + lat.Cl_r*rhat ...
         + lat.Cl_da*d.da + lat.Cl_dr*d.dr;
    Cn = lat.Cn0 + lat.Cn_beta*beta + lat.Cn_p*phat + lat.Cn_r*rhat ...
         + lat.Cn_da*d.da + lat.Cn_dr*d.dr;

    % ---- wind axes -> body axes (FRD) ---------------------------------------
    L = qbar * S * CL;  D = qbar * S * CD;  Y = qbar * S * CY;
    ca = cos(alpha); sa = sin(alpha);
    F = [-D*ca + L*sa;  Y;  -D*sa - L*ca];
    M = qbar * S * [b*Cl;  c*Cm;  b*Cn];

    y = struct('CL', CL, 'CD', CD, 'CY', CY, 'Cl', Cl, 'Cm', Cm, 'Cn', Cn, ...
               'L', L, 'D', D, 'qbar', qbar, 'sigma', sigma, ...
               'stall', abs(alpha) > a_s);
end
