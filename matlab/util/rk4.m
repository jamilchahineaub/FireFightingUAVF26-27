function [t, X] = rk4(f, tspan, x0, dt)
%RK4  Fixed-step Runge-Kutta 4 integrator.
%   [t, X] = rk4(f, [t0 tf], x0, dt)   f = @(t, x) xdot
%   X is N x n (one row per time step).
    t = (tspan(1):dt:tspan(2))';
    n = numel(x0);
    X = zeros(numel(t), n);
    X(1, :) = x0(:)';
    x = x0(:);
    for k = 1:numel(t) - 1
        tk = t(k);
        k1 = f(tk,        x);
        k2 = f(tk + dt/2, x + dt/2 * k1);
        k3 = f(tk + dt/2, x + dt/2 * k2);
        k4 = f(tk + dt,   x + dt   * k3);
        x = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
        X(k + 1, :) = x';
    end
end
