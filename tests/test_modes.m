function [pass, msg] = test_modes()
%TEST_MODES  Sanity checks on the open-loop modes at loaded cruise.
%  - short period is oscillatory with 0.3 <= zeta <= 1.0
%  - phugoid period within 30% of the Lanchester estimate sqrt(2)*pi*V/g
%  - roll subsidence real and stable
%  - dutch roll identified (verdict depends on placeholder lateral data; not a failure)
%  - spiral reported (unstable spiral is expected with Aerosonde placeholders; warn only)
%  Prints the mode table so it appears in the test log.
    P = load_params();
    cfg = struct('V', 20, 'gamma', 0, 'mass', 'loaded', 'prop_model', 'legacy_constant_power');
    [xt, ut, ~] = trim(P, cfg);
    L = linearize(xt, ut, P, cfg);
    T = modes(L, P, struct('category', 'B', 'print', true, 'V', 20));
    get = @(n) T(strcmp({T.name}, n));
    msgs = {};

    sp = get('short period'); ph = get('phugoid'); rl = get('roll subsidence');
    dr = get('dutch roll'); spi = get('spiral');
    ok1 = ~isempty(sp) && sp.zeta >= 0.3 && sp.zeta <= 1.0;
    msgs{end+1} = sprintf('SP wn=%.2f zeta=%.2f', sp.wn, sp.zeta);
    T_lanch = sqrt(2)*pi*20 / P.env.g;
    ok2 = ~isempty(ph) && abs(ph.period - T_lanch) / T_lanch < 0.30;
    msgs{end+1} = sprintf('phugoid T=%.1fs (Lanchester %.1fs) zeta=%.3f', ph.period, T_lanch, ph.zeta);
    ok3 = ~isempty(rl) && real(rl.lambda) < 0;
    msgs{end+1} = sprintf('roll tau=%.2fs', rl.tau);
    ok4 = ~isempty(dr) && ~isempty(spi);
    if ~isempty(spi) && real(spi.lambda) > 0
        msgs{end+1} = sprintf('WARN spiral unstable t2=%.1fs (Aerosonde placeholder lateral data)', spi.t_x);
    end
    pass = ok1 && ok2 && ok3 && ok4;
    msg = strjoin(msgs, '; ');
end
