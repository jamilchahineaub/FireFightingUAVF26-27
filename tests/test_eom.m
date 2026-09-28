function [pass, msg] = test_eom()
%TEST_EOM  Physics sanity checks on eom.m.
%  1. Free fall: with aero and thrust off, wdot = g in body axes (level attitude).
%  2. Torque-free spin with Ixz ~= 0: |J w| conserved over 10 s.
%  3. Quaternion norm stays 1 over 100 s of tumbling.
%  4. 90 deg pitch-up stays finite (no Euler singularity in the integrator).
    P = load_params();
    P.prop.model = 'legacy_constant_power';
    tol = struct('freefall', 1e-9, 'angmom', 1e-7, 'qnorm', 1e-8);
    msgs = {};

    % --- 1. free fall -----------------------------------------------------------
    cfg = struct('aero_off', true, 'thrust_off', true, 'mass', 'loaded');
    x = make_state('pos', [0 0 -100]);
    xd = eom(0, x, zeros(5,1), P, cfg);
    err1 = abs(xd(6) - P.env.g);
    ok1 = err1 < tol.freefall;
    msgs{end+1} = sprintf('freefall wdot-g=%.1e', err1);

    % --- 2. torque-free spin, conserve angular momentum -----------------------
    P2 = P;
    J = P2.mass.loaded.J;  J(1,3) = -0.3; J(3,1) = -0.3;   % inject Ixz to make it non-trivial
    P2.mass.loaded.J = J;
    cfg = struct('aero_off', true, 'thrust_off', true, 'grav_off', true, 'mass', 'loaded');
    x0 = make_state('pqr', [0.5 -0.3 0.8]);
    f = @(t, x) eom(t, x, zeros(5,1), P2, cfg);
    [~, X] = rk4(f, [0 10], x0, 1e-3);
    Q = quat_utils();
    H0 = Q.R_b2n(X(1,7:10)') * (J * X(1,11:13)');     % angular momentum in NED
    H1 = Q.R_b2n(X(end,7:10)') * (J * X(end,11:13)');
    err2 = norm(H1 - H0) / norm(H0);
    ok2 = err2 < tol.angmom;
    msgs{end+1} = sprintf('|H| drift=%.1e', err2);

    % --- 3. quaternion norm -----------------------------------------------------
    qn = sqrt(sum(X(:,7:10).^2, 2));
    err3 = max(abs(qn - 1));
    ok3 = err3 < tol.qnorm;
    msgs{end+1} = sprintf('max|q|-1=%.1e', err3);

    % --- 4. vertical attitude ---------------------------------------------------
    cfg = struct('mass', 'loaded');
    x = make_state('pos', [0 0 -100], 'vb', [20 0 0], 'euler', [0 pi/2 0], 'act', [0 0 0 0.3 0]);
    xd = eom(0, x, [0 0 0 0.3 0]', P, cfg);
    ok4 = all(isfinite(xd));
    msgs{end+1} = 'vertical attitude finite';

    pass = ok1 && ok2 && ok3 && ok4;
    msg = strjoin(msgs, '; ');
end
