function T = modes(L, P, opts)
%MODES  Eigen-analysis of the longitudinal and lateral-directional blocks.
%
%   T = modes(L, P)            L from linearize.m
%   T = modes(L, P, opts)      opts.category = 'B' (cruise) | 'C' (approach)
%                              opts.print = true/false
%                              opts.V = airspeed (for CAP and Froude columns)
%
% Returns a struct array with one row per mode: name, lambda, wn, zeta, tau,
% t_double (unstable) / t_half, period, MIL-F-8785C Level-1 verdict, and the
% Froude-scaled frequency (wn / sqrt(11)) for comparison with full-scale bands.
%
% Mode identification is by eigenvector participation (which states dominate),
% not by frequency ordering, so it is robust when a box wing has an unusually
% well-damped short period.
%
% MIL-F-8785C Level 1, Class I (MIL-F-8785C, 5 Nov 1980, tables IV/VI/VII/VIII
% and para 3.2.1.2):
%   short period  zeta 0.35-1.30 (Cat C), 0.30-2.00 (Cat B)
%   phugoid       zeta >= 0.04
%   roll mode     tau <= 1.0 s (Cat C), 1.4 s (Cat B)
%   spiral        time to double >= 12 s (Cat C), 20 s (Cat B)
%   dutch roll    zeta >= 0.08, zeta*wn >= 0.15 rad/s, wn >= 1.0 (Cat C) / 0.4 (Cat B)

    if nargin < 3, opts = struct(); end
    if ~isfield(opts, 'category'), opts.category = 'B'; end
    if ~isfield(opts, 'print'), opts.print = true; end
    if ~isfield(opts, 'scale'), opts.scale = 11; end     % Froude: full-scale wn = wn/sqrt(scale)
    fr = 1 / sqrt(opts.scale);

    B = bands(opts.category);
    rows = {};

    % ---- longitudinal: drop actuator state for mode identification ----------
    Al = L.lon.A(1:5, 1:5);                       % [u w q theta h]
    [Vl, Dl] = eig(Al);  lam = diag(Dl);
    used = false(size(lam));
    % short period: complex pair with largest |w|,|q| participation
    [sp, used] = pick(lam, Vl, used, [2 3], true);
    [ph, used] = pick(lam, Vl, used, [1 4], true);
    if ~isempty(sp), rows{end+1} = row('short period', sp, B.sp, fr); end
    if ~isempty(ph), rows{end+1} = row('phugoid', ph, B.ph, fr); end
    rest = lam(~used);
    for k = 1:numel(rest)
        if abs(rest(k)) < 1e-2
            rows{end+1} = row('altitude (integr.)', rest(k), [], fr);
        else
            rows{end+1} = row('lon (other)', rest(k), [], fr);
        end
    end

    % ---- lateral-directional -------------------------------------------------
    Aa = L.lat.A(1:5, 1:5);                       % [v p r phi psi]
    Aa = Aa(1:4, 1:4);                            % drop psi (pure integrator)
    [Va, Da] = eig(Aa);  lam = diag(Da);
    used = false(size(lam));
    [dr, used] = pick(lam, Va, used, [1 3], true);
    [rl, used] = pick(lam, Va, used, [2], false);
    [spi, used] = pick(lam, Va, used, [4], false);
    if ~isempty(dr),  rows{end+1} = row('dutch roll', dr, B.dr, fr); end
    if ~isempty(rl),  rows{end+1} = row('roll subsidence', rl, B.roll, fr); end
    if ~isempty(spi), rows{end+1} = row('spiral', spi, B.spiral, fr); end

    T = [rows{:}];

    if opts.print
        fprintf('\n%-16s %-24s %8s %8s %8s %10s %9s  %s\n', 'mode', 'lambda', 'wn', 'zeta', 'tau[s]', 't2/t1/2[s]', 'wn/sqrt11', 'MIL-F-8785C L1');
        for k = 1:numel(T)
            t = T(k);
            fprintf('%-16s %-24s %8.3f %8.3f %8.2f %10.1f %9.3f  %s\n', t.name, ...
                sprintf('%.3f%+.3fi', real(t.lambda), imag(t.lambda)), t.wn, t.zeta, t.tau, t.t_x, t.wn*fr, t.verdict);
        end
        fprintf('  lon/lat coupling = %.1e   step check = %.1e\n', L.coupling, L.step_check);
    end
end

function [lam_sel, used] = pick(lam, V, used, idx, want_complex)
    lam_sel = [];
    best = -1; bi = 0;
    for k = 1:numel(lam)
        if used(k), continue; end
        if want_complex && abs(imag(lam(k))) < 1e-9, continue; end
        if ~want_complex && abs(imag(lam(k))) > 1e-9, continue; end
        v = abs(V(:, k)); v = v / max(norm(v), eps);
        part = sum(v(idx).^2);
        if part > best, best = part; bi = k; end
    end
    if bi > 0
        lam_sel = lam(bi); used(bi) = true;
        if want_complex   % also mark the conjugate
            for k = 1:numel(lam)
                if ~used(k) && abs(lam(k) - conj(lam_sel)) < 1e-9, used(k) = true; end
            end
            if imag(lam_sel) < 0, lam_sel = conj(lam_sel); end
        end
    end
end

function r = row(name, lam, band, fr)
    r.name = name; r.lambda = lam;
    r.wn = abs(lam);
    if abs(imag(lam)) > 1e-9, r.zeta = -real(lam) / abs(lam); else, r.zeta = NaN; end
    r.tau = -1 / real(lam);
    if real(lam) > 0, r.t_x = log(2) / real(lam); else, r.t_x = -log(2) / real(lam); end
    r.period = 2*pi / max(abs(imag(lam)), eps);
    r.wn_froude = r.wn * fr;
    r.verdict = 'n/a';
    if isempty(band), return; end
    ok = true; why = {};
    if isfield(band, 'zeta_min') && ~(r.zeta >= band.zeta_min), ok = false; why{end+1} = sprintf('zeta<%.2f', band.zeta_min); end
    if isfield(band, 'zeta_max') && ~(r.zeta <= band.zeta_max), ok = false; why{end+1} = sprintf('zeta>%.2f', band.zeta_max); end
    if isfield(band, 'tau_max') && ~(r.tau > 0 && r.tau <= band.tau_max), ok = false; why{end+1} = sprintf('tau>%.1fs', band.tau_max); end
    if isfield(band, 't2_min') && real(lam) > 0 && r.t_x < band.t2_min, ok = false; why{end+1} = sprintf('t2<%.0fs', band.t2_min); end
    if isfield(band, 'zwn_min') && ~(r.zeta*r.wn >= band.zwn_min), ok = false; why{end+1} = sprintf('zeta*wn<%.2f', band.zwn_min); end
    if isfield(band, 'wn_min') && ~(r.wn >= band.wn_min), ok = false; why{end+1} = sprintf('wn<%.1f', band.wn_min); end
    if ok, r.verdict = 'Level 1'; else, r.verdict = ['NOT L1: ' strjoin(why, ',')]; end
end

function B = bands(cat)
    switch upper(cat)
        case 'C'
            B.sp = struct('zeta_min', 0.35, 'zeta_max', 1.30);
            B.roll = struct('tau_max', 1.0);
            B.spiral = struct('t2_min', 12);
            B.dr = struct('zeta_min', 0.08, 'zwn_min', 0.15, 'wn_min', 1.0);
        otherwise
            B.sp = struct('zeta_min', 0.30, 'zeta_max', 2.00);
            B.roll = struct('tau_max', 1.4);
            B.spiral = struct('t2_min', 20);
            B.dr = struct('zeta_min', 0.08, 'zwn_min', 0.15, 'wn_min', 0.4);
    end
    B.ph = struct('zeta_min', 0.04);
end
