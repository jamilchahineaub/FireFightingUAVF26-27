function [pass, msg] = test_six_points()
%TEST_SIX_POINTS  All six design points trim and linearise; results saved.
%  Pass: every trim converged with |res| < 1e-8, every short period identified
%  and stable, results/lin_points.mat written with the current params hash.
    P = load_params();
    R = run_six_points('legacy_constant_power', true);
    ok = true; msgs = {};
    for k = 1:numel(R)
        t = R(k).trim;
        sp = R(k).modes(strcmp({R(k).modes.name}, 'short period'));
        oki = t.exitflag > 0 && t.resnorm < 1e-8 && ~isempty(sp) && real(sp.lambda) < 0;
        ok = ok && oki;
        if ~oki, msgs{end+1} = sprintf('%s failed', R(k).id); end
    end
    here = fileparts(mfilename('fullpath'));
    f = fullfile(here, '..', 'results', 'lin_points.mat');
    S = load(f);
    ok = ok && strcmp(S.sha, P.meta.sha256);
    msgs{end+1} = sprintf('%d points, %d stall-blend-affected, saved with sha %s', ...
        numel(R), sum([R.stall_blend_flag]), S.sha(1:12));
    pass = ok;
    msg = strjoin(msgs, '; ');
end
