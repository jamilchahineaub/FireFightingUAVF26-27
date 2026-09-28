function [F, M] = hydro_stub(pos_ned, v_b, quat, pqr, P)
%HYDRO_STUB  Placeholder hydrodynamic force/moment hook (returns zeros).
%
%   [F, M] = hydro_stub(pos_ned, v_b, quat, pqr, P)
%
% Returns zeros. Replace with hull-box buoyancy, damping and added mass for
% the water-landing work. Inputs NED/FRD, outputs FRD about the CG.

    F = zeros(3, 1);
    M = zeros(3, 1);
end
