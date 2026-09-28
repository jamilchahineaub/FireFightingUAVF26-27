function [pass, msg] = test_aero()
%TEST_AERO  Checks on the aerodynamic build-up.
%  1. At alpha = 0, zero rates, zero surfaces: CL = CL0, CD = CD0, Cm = Cm0.
%  2. Lift slope numerically equals CL_alpha in the linear region.
%  3. Stall cap: CL never exceeds ~CL_max by more than the blend overshoot.
%  4. Sign conventions: +de -> Cm decreases; +da -> Cl increases; +dr -> Cn decreases.
%  5. Conventional-elevator equivalence: dL = 0 gives exactly the classic model.
%  6. Static lift = weight check at the reference cruise CL.
    P = load_params();
    rho = isa_density(P.trim_ref.altitude_m, P);
    d0 = struct('de', 0, 'da', 0, 'dr', 0, 'dL', 0);
    lon = P.aero.lon;
    msgs = {};

    [~, ~, y] = aero(20, 0, 0, [0 0 0], d0, rho, P);
    ok1 = abs(y.CL - lon.CL0) < 1e-6 && abs(y.CD - (P.aero.CD0 + P.aero.K*lon.CL0^2)) < 1e-6 ...
          && abs(y.Cm - lon.Cm0) < 1e-6;
    msgs{end+1} = sprintf('alpha=0: CL=%.4f CD=%.4f Cm=%.4f', y.CL, y.CD, y.Cm);

    da = 1e-4;
    [~, ~, ya] = aero(20, 0.05 + da, 0, [0 0 0], d0, rho, P);
    [~, ~, yb] = aero(20, 0.05 - da, 0, [0 0 0], d0, rho, P);
    slope = (ya.CL - yb.CL) / (2*da);
    ok2 = abs(slope - lon.CL_alpha) / lon.CL_alpha < 1e-3;
    msgs{end+1} = sprintf('CL_alpha num=%.3f', slope);

    CLs = zeros(1, 60); al = linspace(0, deg2rad(30), 60);
    for k = 1:60, [~, ~, yk] = aero(20, al(k), 0, [0 0 0], d0, rho, P); CLs(k) = yk.CL; end
    [CLpk, ipk] = max(CLs);
    ok3 = abs(CLpk - P.aero.CL_max) / P.aero.CL_max < 0.02;
    msgs{end+1} = sprintf('peak CL=%.3f at %.1f deg (CL_max %.2f, blend centre %.1f deg)', ...
        CLpk, rad2deg(al(ipk)), P.aero.CL_max, rad2deg(P.aero.alpha_stall));

    de = struct('de', 0.05, 'da', 0, 'dr', 0, 'dL', 0);
    dA = struct('de', 0, 'da', 0.05, 'dr', 0, 'dL', 0);
    dR = struct('de', 0, 'da', 0, 'dr', 0.05, 'dL', 0);
    [~, ~, ye] = aero(20, 0.05, 0, [0 0 0], de, rho, P);
    [~, ~, yA] = aero(20, 0.05, 0, [0 0 0], dA, rho, P);
    [~, ~, yR] = aero(20, 0.05, 0, [0 0 0], dR, rho, P);
    [~, ~, y0] = aero(20, 0.05, 0, [0 0 0], d0, rho, P);
    ok4 = (ye.Cm < y0.Cm) && (yA.Cl > y0.Cl) && (yR.Cn < y0.Cn) && (ye.CL > y0.CL);
    msgs{end+1} = 'signs de/da/dr ok';

    % conventional elevator equivalence (dL=0): Cm = Cm0 + Cma*a + Cmq*qhat + Cmde*de
    a = 0.06; q = 0.2; Va = 18;
    [~, ~, yc] = aero(Va, a, 0, [0 q 0], de, rho, P);
    Cm_ref = lon.Cm0 + lon.Cm_alpha*a + lon.Cm_q*q*P.geom.c_bar/(2*Va) + lon.Cm_de*de.de;
    ok5 = abs(yc.Cm - Cm_ref) < 1e-12;
    msgs{end+1} = sprintf('conv. elevator equiv err=%.1e', abs(yc.Cm - Cm_ref));

    % lift = weight at reference CL
    tr = P.trim_ref.cruise_loaded;
    W = P.mass.loaded.m * P.env.g;
    CL_req = W / (0.5 * rho * tr.V^2 * P.geom.S);
    ok6 = abs(CL_req - tr.CL) / tr.CL < 0.01;
    msgs{end+1} = sprintf('CL for L=W: %.3f (ref %.3f)', CL_req, tr.CL);

    pass = ok1 && ok2 && ok3 && ok4 && ok5 && ok6;
    msg = strjoin(msgs, '; ');
end
