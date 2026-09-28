function [pass, msg] = test_actuator()
%TEST_ACTUATOR  First-order lag time constant, position and rate saturation.
    P = load_params();
    msgs = {};

    % time constant: step of 5 deg on elevator, 63.2% at t = tau
    tau = P.act.elevator.tau;
    cmd = [deg2rad(5) 0 0 0 0]';
    f = @(t, d) actuator(d, cmd, P);
    [t, D] = rk4(f, [0 5*tau], zeros(5,1), tau/200);
    [~, i] = min(abs(t - tau));
    frac = D(i, 1) / cmd(1);
    ok1 = abs(frac - (1 - exp(-1))) < 0.02;
    msgs{end+1} = sprintf('elevator 63.2%% at tau: %.3f', frac);

    % position saturation
    [~, dsat] = actuator(zeros(5,1), [deg2rad(60) deg2rad(-60) deg2rad(60) 1.5 deg2rad(40)]', P);
    ok2 = abs(dsat(1) - P.act.elevator.delta_max) < 1e-12 && ...
          abs(dsat(2) + P.act.aileron.delta_max) < 1e-12 && ...
          abs(dsat(3) - P.act.rudder.delta_max) < 1e-12 && ...
          abs(dsat(4) - 1.0) < 1e-12;
    msgs{end+1} = 'position saturation ok';

    % rate saturation: big step -> ddot equals rate_max
    dd = actuator(zeros(5,1), [P.act.elevator.delta_max 0 0 0 0]', P);
    ok3 = abs(dd(1) - min(P.act.elevator.rate_max, P.act.elevator.delta_max / tau)) < 1e-12;
    msgs{end+1} = sprintf('rate-limited ddot=%.1f deg/s', rad2deg(dd(1)));

    pass = ok1 && ok2 && ok3;
    msg = strjoin(msgs, '; ');
end
