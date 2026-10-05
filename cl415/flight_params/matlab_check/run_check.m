% run the team's 6-dof tools (FireFightingUAVF26-27/matlab, unchanged) on the cl415 parameter set:
% trim, linearise and mode analysis at the six design points, MIL-F-8785C Level 1 verdicts.
%   matlab -batch "run('flight_params/matlab_check/run_check.m')"      (from the cl415 folder)
here = fileparts(mfilename('fullpath'));
team = fullfile(here, '..', '..', '..', 'matlab');
addpath(team); addpath(fullfile(team, 'util'));
if ~exist('fsolve', 'file'), addpath(fullfile(here, 'shim')); end   % fsolve stand-in when the toolbox is missing
out = fullfile(here, 'check_output.txt');
if exist(out, 'file'), delete(out); end
diary(out);
P = load_params(fullfile(here, '..', 'cl415_params.json'));
fprintf('cl415 params sha %s, CL_max %.3f, linear stall alpha %.2f deg, blend centre %.2f deg\n', ...
    P.meta.sha256(1:12), P.aero.CL_max, rad2deg(P.aero.alpha_stall_lin), rad2deg(P.aero.alpha_stall));
dp = P.design_points;
res = struct('id', {}, 'alpha_deg', {}, 'de_deg', {}, 'dT', {}, 'T', {}, 'exitflag', {}, 'modes', {});
for k = 1:numel(dp)
    d = dp(k);
    cfg = struct('V', d.V, 'gamma', deg2rad(d.gamma_deg), 'mass', d.mass);
    [xt, ut, info] = trim(P, cfg);
    L = linearize(xt, ut, P, cfg);
    if strcmpi(d.basis, 'cruise'), cat = 'B'; else, cat = 'C'; end
    fprintf('\n==== %s  %s  V = %.1f m/s  gamma = %.1f deg ====\n', d.id, d.mass, d.V, d.gamma_deg);
    fprintf('trim: alpha %.2f deg, de %.2f deg, dT %.3f, T %.2f N, CL %.3f, exitflag %d\n', ...
        info.alpha_deg, info.de_deg, info.dT, info.T, info.CL, info.exitflag);
    M = modes(L, P, struct('category', cat, 'print', true, 'V', d.V));
    for j = 1:numel(M), M(j).lambda = [real(M(j).lambda), imag(M(j).lambda)]; end   % json has no complex
    res(end+1) = struct('id', d.id, 'alpha_deg', info.alpha_deg, 'de_deg', info.de_deg, 'dT', info.dT, ...
                        'T', info.T, 'exitflag', info.exitflag, 'modes', M); %#ok<SAGROW>
end
diary off;
fid = fopen(fullfile(here, 'check_results.json'), 'w');
fprintf(fid, '%s', jsonencode(res));
fclose(fid);
