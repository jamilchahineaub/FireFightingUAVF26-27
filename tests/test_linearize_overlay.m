function [pass, msg] = test_linearize_overlay()
%TEST_LINEARIZE_OVERLAY  Nonlinear vs linear response to small doublets at cruise.
%
%  1 deg elevator doublet (1 s up, 1 s down from t = 1 s), 20 s, compare q, theta, V, h
%  2 deg aileron doublet, 20 s, compare p, phi, r
%  Pass: normalised RMS error <= 5% on q, theta, p, phi, r (with absolute floors),
%        <= 10% on V and h (phugoid drifts nonlinearly). Also: Jacobian step
%        check < 1e-6, lon/lat coupling ~ 0, unloaded M_dT within 10% of +1.02.
    P = load_params();
    cfg = struct('V', 20, 'gamma', 0, 'mass', 'loaded', 'prop_model', 'legacy_constant_power');
    [xt, ut, ~] = trim(P, cfg);
    L = linearize(xt, ut, P, cfg);
    Q = quat_utils();
    msgs = {};

    dt = 5e-3; tf = 20; t = (0:dt:tf)';
    doublet = @(t, t0) (t >= t0 & t < t0+1) - (t >= t0+1 & t < t0+2);

    % ---- elevator doublet ------------------------------------------------------
    amp = deg2rad(1);
    u_of_t = @(tt) ut + [amp*doublet(tt,1); 0; 0; 0; 0];
    ecfg = struct('mass', cfg.mass, 'prop_model', cfg.prop_model);
    [~, Xn] = rk4(@(tt, x) eom(tt, x, u_of_t(tt), P, ecfg), [0 tf], xt, dt);
    Zl = lsim_rk4(L.A, L.B, @(tt) u_of_t(tt) - ut, t);          % perturbations in z-coords
    % nonlinear outputs
    th_n = zeros(size(t)); for k = 1:numel(t), [~, th_n(k)] = Q.to_euler(Xn(k,7:10)'); end
    q_n = Xn(:,12); V_n = sqrt(sum(Xn(:,4:6).^2, 2)); h_n = -Xn(:,3);
    % linear outputs (perturbation + trim)
    [~, th0] = Q.to_euler(xt(7:10));
    th_l = th0 + Zl(:,8); q_l = Zl(:,11);
    V0 = norm(xt(4:6)); V_l = V0 + (xt(4)*Zl(:,4) + xt(6)*Zl(:,6)) / V0; h_l = -xt(3) - Zl(:,3);
    e_q  = nrms(q_n, q_l, 0, deg2rad(0.05));
    e_th = nrms(th_n, th_l, th0, deg2rad(0.1));
    e_V  = nrms(V_n, V_l, V0, 0.05);
    e_h  = nrms(h_n, h_l, -xt(3), 0.2);
    okE = e_q <= 0.05 && e_th <= 0.05 && e_V <= 0.10 && e_h <= 0.10;
    msgs{end+1} = sprintf('elev doublet NRMS q=%.1f%% th=%.1f%% V=%.1f%% h=%.1f%%', 100*e_q, 100*e_th, 100*e_V, 100*e_h);

    % ---- aileron doublet -------------------------------------------------------
    amp = deg2rad(2);
    u_of_t = @(tt) ut + [0; amp*doublet(tt,1); 0; 0; 0];
    [~, Xn] = rk4(@(tt, x) eom(tt, x, u_of_t(tt), P, ecfg), [0 tf], xt, dt);
    Zl = lsim_rk4(L.A, L.B, @(tt) u_of_t(tt) - ut, t);
    ph_n = zeros(size(t)); for k = 1:numel(t), ph_n(k) = Q.to_euler(Xn(k,7:10)'); end
    p_n = Xn(:,11); r_n = Xn(:,13);
    ph_l = Zl(:,7); p_l = Zl(:,10); r_l = Zl(:,12);
    e_p  = nrms(p_n, p_l, 0, deg2rad(0.05));
    e_ph = nrms(ph_n, ph_l, 0, deg2rad(0.2));
    e_r  = nrms(r_n, r_l, 0, deg2rad(0.05));
    okA = e_p <= 0.05 && e_ph <= 0.05 && e_r <= 0.05;
    msgs{end+1} = sprintf('ail doublet NRMS p=%.1f%% phi=%.1f%% r=%.1f%%', 100*e_p, 100*e_ph, 100*e_r);

    % ---- Jacobian quality and the M_dT regression ------------------------------
    okJ = L.step_check < 1e-6 && L.coupling < 1e-6;
    cfgU = cfg; cfgU.mass = 'unloaded';
    [xu, uu, ~] = trim(P, cfgU); Lu = linearize(xu, uu, P, cfgU);
    okM = abs(Lu.M_dT - 1.02) / 1.02 < 0.10 && abs(L.M_dT) < 1e-6;
    msgs{end+1} = sprintf('step check %.1e, coupling %.1e, M_dT loaded %.3f / unloaded %.3f (report 0 / +1.02)', ...
        L.step_check, L.coupling, L.M_dT, Lu.M_dT);

    pass = okE && okA && okJ && okM;
    msg = strjoin(msgs, '; ');
end

function e = nrms(yn, yl, y0, floor_abs)
    d = yn - yl;
    scale = max(max(abs(yn - y0)), floor_abs);
    e = sqrt(mean(d.^2)) / scale;
end

function Z = lsim_rk4(A, B, ufun, t)
    n = size(A, 1); Z = zeros(numel(t), n); z = zeros(n, 1); dt = t(2) - t(1);
    f = @(tt, z) A*z + B*ufun(tt);
    for k = 1:numel(t)-1
        tk = t(k);
        k1 = f(tk, z); k2 = f(tk+dt/2, z+dt/2*k1); k3 = f(tk+dt/2, z+dt/2*k2); k4 = f(tk+dt, z+dt*k3);
        z = z + dt/6*(k1+2*k2+2*k3+k4); Z(k+1,:) = z';
    end
end
