function L = linearize(xt, ut, P, cfg)
%LINEARIZE  Numerical Jacobian of eom.m at a trim point, in Euler-angle coordinates.
%
%   L = linearize(xt, ut, P, cfg)     xt, ut from trim.m; cfg.mass, cfg.prop_model
%
% The 18-state quaternion model is reparametrised with 17 states for the
% Jacobian (quaternion -> Euler perturbation), because perturbing quaternion
% components directly gives a rank-deficient block.
%
%   z = [pN pE pD u v w phi theta psi p q r de da dr dT dL]   (17)
%   inputs  = [de_cmd da_cmd dr_cmd dT_cmd dL_cmd]            (5)
%
% Central differences with per-state step sizes, plus a step-halving check
% (L.step_check: max relative change of A between h and h/10).
%
% Sub-blocks returned (Beard & McLain ch. 5 ordering):
%   L.lon.A, L.lon.B   states [u w q theta h de_act]  inputs [de_cmd dT_cmd]
%   L.lat.A, L.lat.B   states [v p r phi psi da_act dr_act] inputs [da_cmd dr_cmd]
%   L.lon.states, L.lat.states  cell arrays of names
%   L.coupling                  norm of the lon<->lat off-diagonal blocks
% plus the full A (17x17), B (17x5).

    Q = quat_utils();
    if ~isfield(cfg, 'prop_model'), cfg.prop_model = P.prop.model; end
    ecfg = struct('mass', cfg.mass, 'prop_model', cfg.prop_model);

    z0 = to_z(xt, Q);
    f  = @(z, u) to_zdot(z, u, P, ecfg, Q);

    n = 17; m = 5;
    hz = [1e-3*ones(3,1); 1e-4*ones(3,1); 1e-6*ones(3,1); 1e-5*ones(3,1); 1e-6*ones(5,1)];
    hu = [1e-6; 1e-6; 1e-6; 1e-5; 1e-6];

    A  = jac(@(z) f(z, ut), z0, hz);
    A2 = jac(@(z) f(z, ut), z0, hz / 10);
    B  = jac(@(u) f(z0, u), ut(:), hu);

    L.A = A; L.B = B;
    L.step_check = max(max(abs(A - A2))) / max(1, max(max(abs(A))));
    L.z0 = z0; L.u0 = ut(:);
    L.states = {'pN','pE','pD','u','v','w','phi','theta','psi','p','q','r','de','da','dr','dT','dL'};
    L.inputs = {'de_cmd','da_cmd','dr_cmd','dT_cmd','dL_cmd'};

    % ---- longitudinal block: [u w q theta h de_act dT_act], h = -pD ---------
    iu=4; iw=6; iq=11; ith=8; ipD=3; ide=13; idT=16;
    idx_lon = [iu iw iq ith ipD ide idT];
    Alon = A(idx_lon, idx_lon);
    Blon = B(idx_lon, [1 4]);
    % convert pD -> h: row and column sign flips
    S = diag([1 1 1 1 -1 1 1]);
    L.lon.A = S * Alon * S;
    L.lon.B = S * Blon;
    L.lon.states = {'u','w','q','theta','h','de_act','dT_act'};
    L.lon.inputs = {'de_cmd','dT_cmd'};

    % ---- legacy 6-state form: throttle acts directly (May-2026 study) --------
    % Eliminate dT_act by its steady state (dT_act = dT_cmd): the column of A
    % for dT_act becomes the B column for dT_cmd.
    L.lon6.A = L.lon.A(1:6, 1:6);
    L.lon6.B = [L.lon.B(1:6, 1), L.lon.A(1:6, 7)];
    L.lon6.states = {'u','w','q','theta','h','de_act'};
    L.lon6.inputs = {'de_cmd','dT'};
    L.M_dT = L.lon6.B(3, 2);       % pitch acceleration per unit throttle (report: +1.02 unloaded)

    % ---- lateral block: [v p r phi psi da_act dr_act] ------------------------
    iv=5; ip=10; ir=12; iph=7; ips=9; ida=14; idr=15;
    idx_lat = [iv ip ir iph ips ida idr];
    L.lat.A = A(idx_lat, idx_lat);
    L.lat.B = B(idx_lat, [2 3]);
    L.lat.states = {'v','p','r','phi','psi','da_act','dr_act'};
    L.lat.inputs = {'da_cmd','dr_cmd'};

    % ---- coupling measure -----------------------------------------------------
    L.coupling = max(norm(A(idx_lon, idx_lat)), norm(A(idx_lat, idx_lon))) / norm(A);
    L.idx_lon = idx_lon; L.idx_lat = idx_lat;
end

function J = jac(fun, x0, h)
    n = numel(x0); f0 = fun(x0); J = zeros(numel(f0), n);
    for j = 1:n
        e = zeros(n,1); e(j) = h(j);
        J(:, j) = (fun(x0 + e) - fun(x0 - e)) / (2*h(j));
    end
end

function z = to_z(x, Q)
    [phi, theta, psi] = Q.to_euler(x(7:10));
    z = [x(1:6); phi; theta; psi; x(11:18)];
end

function x = to_x(z, Q)
    quat = Q.from_euler(z(7), z(8), z(9));
    x = [z(1:6); quat(:); z(10:17)];
end

function zd = to_zdot(z, u, P, ecfg, Q)
    x = to_x(z, Q);
    xd = eom(0, x, u, P, ecfg);
    % Euler-angle rates from body rates (B&M eq. 3.3)
    phi = z(7); th = z(8);
    p = z(10); q = z(11); r = z(12);
    E = [1, sin(phi)*tan(th), cos(phi)*tan(th);
         0, cos(phi),        -sin(phi);
         0, sin(phi)/cos(th), cos(phi)/cos(th)];
    eul_dot = E * [p; q; r];
    zd = [xd(1:6); eul_dot; xd(11:18)];
end
