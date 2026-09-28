function ok = run_all_tests(scope)
%RUN_ALL_TESTS  Run the model / trim / linearisation test suite and print PASS/FAIL.
%
%   run_all_tests()            same as 'all'
%   run_all_tests('day1')      model, trim, linearise, modes
%   run_all_tests('all')       day1 + six-point batch
%
% Works in MATLAB and GNU Octave (no toolboxes needed). Prints the params.yaml
% hash so every log can be tied to the exact parameter set.

    if nargin < 1, scope = 'all'; end
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, '..', 'matlab'));
    addpath(fullfile(here, '..', 'matlab', 'util'));

    day1 = {@test_eom, @test_aero, @test_propulsion, @test_actuator, ...
            @test_trim_cruise, @test_linearize_overlay, @test_modes};
    day2 = {@test_six_points, @test_sdf_matches_matlab};
    switch lower(scope)
        case 'day1', tests = day1;
        otherwise,   tests = [day1, day2];
    end

    P = load_params();
    fprintf('\n==== prandtl-uav test suite (%s) ====\n', scope);
    fprintf('params.yaml sha256 = %s   schema %d   %s\n\n', P.meta.sha256, P.meta.schema_version, datestr(now));
    ok = false(1, numel(tests));
    t0 = tic;
    for k = 1:numel(tests)
        name = func2str(tests{k});
        tk = tic;
        try
            [pass, msg] = tests{k}();
        catch e
            pass = false; msg = ['ERROR: ' e.message];
        end
        ok(k) = pass;
        if pass, s = 'PASS'; else, s = 'FAIL'; end
        fprintf('%-26s %s  (%.1fs)\n    %s\n', name, s, toc(tk), msg);
    end
    fprintf('\n%d/%d passed in %.0f s\n', sum(ok), numel(ok), toc(t0));
    if ~all(ok), fprintf('*** FAILURES ***\n'); end
end
