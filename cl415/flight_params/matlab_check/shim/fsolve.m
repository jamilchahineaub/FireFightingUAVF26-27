function [z, fval, exitflag] = fsolve(fun, z0, opts)
%FSOLVE  stand-in for the optimization toolbox fsolve (not installed here), only for run_check.m.
% levenberg-marquardt on a square system with a central-difference jacobian. same call form as the
% team's trim.m uses: [z, fval, exitflag] = fsolve(fun, z0, optimset(...)).
    if nargin < 3, opts = struct(); end
    tolF = getopt(opts, 'TolFun', 1e-12);
    tolX = getopt(opts, 'TolX', 1e-12);
    maxIt = getopt(opts, 'MaxIter', 400);
    z = z0(:);
    f = fun(z);
    lam = 1e-3;
    exitflag = 0;
    for it = 1:maxIt
        n = numel(z);
        J = zeros(numel(f), n);
        for k = 1:n
            h = 1e-6 * max(1, abs(z(k)));
            e = zeros(n, 1); e(k) = h;
            J(:, k) = (fun(z + e) - fun(z - e)) / (2 * h);
        end
        while true
            dz = -(J' * J + lam * diag(diag(J' * J) + 1e-12)) \ (J' * f);
            fn = fun(z + dz);
            if norm(fn) < norm(f)
                z = z + dz; f = fn; lam = max(lam / 5, 1e-12);
                break
            end
            lam = lam * 10;
            if lam > 1e12, break; end
        end
        if norm(f) < tolF || norm(dz) < tolX * (1 + norm(z))
            exitflag = 1;
            break
        end
        if lam > 1e12, exitflag = -2; break; end
    end
    fval = f;
end

function v = getopt(o, name, def)
    if isstruct(o) && isfield(o, name) && ~isempty(o.(name)), v = o.(name); else, v = def; end
end
