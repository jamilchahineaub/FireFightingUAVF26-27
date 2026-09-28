function R = run_six_points(prop_model, save_results)
%RUN_SIX_POINTS  Trim + linearise + modes at the six design points of the plan.
%
%   R = run_six_points()                   uses P.prop.model, saves results/lin_points.mat
%   R = run_six_points('momentum', false)
%
% Design points come from params.yaml (design_points): C14 A14 S14 C10 A10 S10.
% For each: trim info, linear blocks, mode table (Cat B for cruise, Cat C for
% approach and slow flight), transfer functions, and a flag if the trim alpha
% is within 2 deg of the linear stall angle (blend-affected).

    P = load_params();
    if nargin < 1 || isempty(prop_model), prop_model = P.prop.model; end
    if nargin < 2, save_results = true; end
    dp = P.design_points;
    R = struct('id', {}, 'trim', {}, 'L', {}, 'modes', {}, 'G', {}, 'stall_blend_flag', {});

    fprintf('\n=== six design points, propulsion = %s, params sha %s ===\n', prop_model, P.meta.sha256(1:12));
    fprintf('%-4s %-9s %5s %6s | %7s %7s %7s %7s | %s\n', 'id', 'mass', 'V', 'gamma', 'alpha', 'de', 'dT', 'T[N]', 'note');
    for k = 1:numel(dp)
        d = dp(k);
        cfg = struct('V', d.V, 'gamma', deg2rad(d.gamma_deg), 'mass', d.mass, 'prop_model', prop_model);
        [xt, ut, info] = trim(P, cfg);
        L = linearize(xt, ut, P, cfg);
        if strcmpi(d.basis, 'cruise'), cat = 'B'; else, cat = 'C'; end
        note = '';
        flag = info.alpha > P.aero.alpha_stall_lin - deg2rad(2);
        if flag, note = 'stall-blend-affected'; end
        if info.exitflag <= 0, note = [note ' TRIM NOT CONVERGED']; end
        fprintf('%-4s %-9s %5.1f %6.1f | %7.2f %7.2f %7.3f %7.2f | %s\n', d.id, d.mass, d.V, d.gamma_deg, ...
                info.alpha_deg, info.de_deg, info.dT, info.T, note);
        M = modes(L, P, struct('category', cat, 'print', false));
        G = tf_extract(L);
        R(end+1) = struct('id', d.id, 'trim', info, 'L', L, 'modes', M, 'G', G, 'stall_blend_flag', flag);
    end

    % mode summary table
    fprintf('\n%-4s | %-22s | %-22s | %-22s | %-10s | %-12s\n', 'id', 'short period', 'phugoid', 'dutch roll', 'roll tau', 'spiral t2');
    for k = 1:numel(R)
        M = R(k).modes; g = @(n) M(strcmp({M.name}, n));
        sp = g('short period'); ph = g('phugoid'); dr = g('dutch roll'); rl = g('roll subsidence'); spi = g('spiral');
        fprintf('%-4s | %s | %s | %s | %-10s | %-12s\n', R(k).id, fmtm(sp), fmtm(ph), fmtm(dr), ...
                sprintf('%.2fs', rl.tau), sprintf('%.1fs%s', spi.t_x, ternary(real(spi.lambda) > 0, ' (unst)', '')));
    end

    if save_results
        here = fileparts(mfilename('fullpath'));
        out = fullfile(here, '..', 'results');
        if ~exist(out, 'dir'), mkdir(out); end
        sha = P.meta.sha256;
        save(fullfile(out, 'lin_points.mat'), 'R', 'sha', 'prop_model', '-v7');
        fprintf('\nsaved results/lin_points.mat\n');
    end
end

function s = fmtm(m)
    if isempty(m), s = sprintf('%-22s', '-'); return; end
    s = sprintf('wn %5.2f z %5.2f %-6s', m.wn, m.zeta, ternary(strncmp(m.verdict, 'Level', 5), 'L1', 'notL1'));
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
