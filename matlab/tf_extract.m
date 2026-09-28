function G = tf_extract(L, opts)
%TF_EXTRACT  Design transfer functions from the linear blocks (no toolbox needed).
%
%   G = tf_extract(L)          L from linearize.m
%   G = tf_extract(L, struct('print', true))
%
% Returns numerator/denominator polynomial pairs (descending powers of s) for
%   G.q_de     pitch rate / elevator command       (actuator included)
%   G.th_de    pitch angle / elevator command
%   G.p_da     roll rate / aileron command
%   G.phi_da   roll angle / aileron command
%   G.r_dr     yaw rate / rudder command
%   G.V_dT     airspeed / throttle command
%   G.h_th     altitude / pitch angle  (from the [u w q theta h] block: h_dot ~ V*theta - w)
% and for each: dc gain, poles, zeros, rhp_zeros flag.
%
% Uses the identity  C (sI-A)^-1 B = [det(sI-A+BC) - det(sI-A)] / det(sI-A)
% so it runs in core MATLAB/Octave without the Control System Toolbox.
% With the toolbox: tf(G.q_de.num, G.q_de.den).

    if nargin < 2, opts = struct(); end
    if ~isfield(opts, 'print'), opts.print = false; end

    % Longitudinal, lon states [u w q theta h de_act dT_act].
    % Pitch TFs: keep [u w q theta de_act] (drop h integrator and throttle actuator)
    ip = [1 2 3 4 6];
    A = L.lon.A(ip, ip); B = L.lon.B(ip, 1); n = numel(ip);
    C = @(i) full(sparse(1, i, 1, 1, n));
    G.q_de  = siso(A, B, C(3));
    G.th_de = siso(A, B, C(4));
    % altitude from elevator: keep h
    ih = [1 2 3 4 5 6];
    A = L.lon.A(ih, ih); B = L.lon.B(ih, 1); n = numel(ih);
    C = @(i) full(sparse(1, i, 1, 1, n));
    G.h_de  = siso(A, B, C(5));
    % throttle TFs: keep [u w q theta h dT_act] (drop elevator actuator)
    it = [1 2 3 4 5 7];
    A = L.lon.A(it, it); B = L.lon.B(it, 2); n = numel(it);
    C = @(i) full(sparse(1, i, 1, 1, n));
    % airspeed perturbation ~ (u0*du + w0*dw)/V0
    u0 = L.z0(4); w0 = L.z0(6); V0 = hypot(u0, w0);
    G.V_dT  = siso(A, B, [u0/V0, w0/V0, 0, 0, 0, 0]);
    G.h_dT  = siso(A, B, C(5));

    % Lateral, lat states [v p r phi psi da_act dr_act]; drop psi integrator.
    ia = [1 2 3 4 6];      % aileron TFs
    A = L.lat.A(ia, ia); B = L.lat.B(ia, 1); n = numel(ia);
    C = @(i) full(sparse(1, i, 1, 1, n));
    G.p_da   = siso(A, B, C(2));
    G.phi_da = siso(A, B, C(4));
    G.r_da   = siso(A, B, C(3));
    ir = [1 2 3 4 7];      % rudder TFs
    A = L.lat.A(ir, ir); B = L.lat.B(ir, 2); n = numel(ir);
    C = @(i) full(sparse(1, i, 1, 1, n));
    G.r_dr   = siso(A, B, C(3));
    G.beta_dr = siso(A, B, [1/V0, 0, 0, 0, 0]);

    if opts.print
        f = fieldnames(G);
        fprintf('\n%-8s %10s  %s\n', 'TF', 'dc gain', 'poles (rad/s) / zeros');
        for k = 1:numel(f)
            g = G.(f{k});
            fprintf('%-8s %10.3g  p: %s\n%-20s z: %s%s\n', f{k}, g.dc, fmt(g.poles), '', fmt(g.zeros), ...
                    ternary(g.rhp_zeros, '  [RHP zero]', ''));
        end
    end
end

function g = siso(A, b, c)
    den = real(poly(A));
    num = real(poly(A - b*c) - den);
    % strip leading zeros of num
    i = find(abs(num) > 1e-12 * max(abs(num)), 1);
    if isempty(i), num = 0; else, num = num(i:end); end
    g.num = num; g.den = den;
    g.poles = roots(den); g.zeros = roots(num);
    g.rhp_zeros = any(real(g.zeros) > 1e-9);
    if abs(den(end)) > 1e-12, g.dc = num(end) / den(end); else, g.dc = Inf; end
end

function s = fmt(v)
    if isempty(v), s = '-'; return; end
    parts = cell(1, numel(v));
    for k = 1:numel(v)
        if abs(imag(v(k))) < 1e-9, parts{k} = sprintf('%.3g', real(v(k)));
        else, parts{k} = sprintf('%.3g%+.3gi', real(v(k)), imag(v(k))); end
    end
    s = strjoin(parts, ', ');
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
