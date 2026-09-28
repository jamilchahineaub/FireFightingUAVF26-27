function rho = isa_density(h, P)
%ISA_DENSITY  Air density at altitude h [m] (ISA troposphere).
%   rho = isa_density(h, P)   uses P.env.rho_sl and P.env.isa flag.
    if nargin > 1 && isfield(P, 'env') && isfield(P.env, 'isa') && ~P.env.isa
        rho = P.env.rho_sl; return;
    end
    T0 = 288.15; L = 0.0065; g0 = 9.80665; R = 287.058;
    rho0 = 1.225;
    if nargin > 1 && isfield(P, 'env'), rho0 = P.env.rho_sl; end
    T = T0 - L * h;
    rho = rho0 * (T / T0)^(g0 / (R * L) - 1);
end
