function M = mass_props(raw)
%MASS_PROPS  Assemble mass, CG and inertia tensors for both configurations.
%
%   M = mass_props(raw)   raw = jsondecode of params.json
%
% Returns M.loaded and M.unloaded, each with
%   m        total mass [kg]
%   cg       CG position in FRD relative to the AIRFRAME CG [m]  (unloaded: [0 0 0])
%   J        3x3 inertia tensor about the configuration CG, FRD axes,
%            J = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz]
%   r_thrust thrust-line offset from the configuration CG, FRD [m]
%
% Mass-station model (May-2026 study): airframe as one body with radii of
% gyration kx = 0.25 b, ky = 0.30 L_fus, kz = sqrt(kx^2+ky^2); payload as a
% point mass 0.12 m below the airframe CG. Parallel-axis theorem gives the
% loaded tensor. The thrust line passes through the LOADED CG by design, so
% after the drop it sits below the unloaded CG (r_thrust.z > 0 in FRD).
%
% When SolidWorks delivers the tensor: set mass_properties.use_cad: true in
% params.yaml and fill mass_properties.cad.*; this function then uses those.

    mp = raw.mass_properties;
    g  = raw.geometry;

    if mp.use_rounded_mass
        m_af = mp.airframe.m_rounded;
        m_pl = mp.loaded.m_rounded - m_af;
    else
        m_af = mp.airframe.m;
        m_pl = mp.payload.m;
    end

    if isfield(mp, 'use_cad') && mp.use_cad
        for cfg = {'loaded', 'unloaded'}
            c = mp.cad.(cfg{1});
            M.(cfg{1}).m  = c.m;
            M.(cfg{1}).cg = c.cg_frd(:)';
            M.(cfg{1}).J  = [c.Ixx 0 -c.Ixz; 0 c.Iyy 0; -c.Ixz 0 c.Izz];
        end
    else
        kx = mp.airframe.kx_over_b * g.b;
        ky = mp.airframe.ky_over_Lfus * g.L_fus;
        kz = sqrt(kx^2 + ky^2);
        Ixz = mp.Ixz_airframe;
        J_af = [m_af*kx^2, 0, -Ixz; 0, m_af*ky^2, 0; -Ixz, 0, m_af*kz^2];

        % unloaded = airframe alone, CG at airframe CG
        M.unloaded.m  = m_af;
        M.unloaded.cg = [0 0 0];
        M.unloaded.J  = J_af;

        % loaded = airframe + payload point mass
        r_pl = mp.payload.r_from_airframe_cg_frd(:)';        % [0 0 +0.12] FRD
        m_tot = m_af + m_pl;
        cg = (m_pl * r_pl) / m_tot;                           % [0 0 +0.0343]
        d_af = -cg;                                           % airframe CG rel. to combined CG
        d_pl = r_pl - cg;
        J = J_af + m_af * shift(d_af) + m_pl * shift(d_pl);
        M.loaded.m  = m_tot;
        M.loaded.cg = cg;
        M.loaded.J  = J;
    end

    % thrust line: through the loaded CG. Offset of each configuration's CG
    % from the thrust line, expressed as r_thrust = (thrust point) - (CG).
    r_line = raw.propulsion.thrust_line.r_thrust_from_loaded_cg_frd(:)' + M.loaded.cg;  % thrust point rel. airframe CG
    M.loaded.r_thrust   = r_line - M.loaded.cg;      % [0 0 0]
    M.unloaded.r_thrust = r_line - M.unloaded.cg;    % [0 0 +0.0343]  -> nose-up moment

    M.m_payload = m_pl;
    M.kx = 0; M.ky = 0;
    if ~(isfield(mp, 'use_cad') && mp.use_cad), M.kx = kx; M.ky = ky; M.kz = kz; end
end

function S = shift(d)
% parallel-axis term for a point at offset d: m*( (d.d) I - d d^T )
    S = (d*d') * eye(3) - (d' * d);
end
