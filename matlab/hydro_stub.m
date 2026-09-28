function [F, M] = hydro_stub(pos_ned, v_b, quat, pqr, P)
%HYDRO_STUB  Placeholder hydrodynamic force/moment hook (returns zeros).
%
%   [F, M] = hydro_stub(pos_ned, v_b, quat, pqr, P)
%
% eom.m calls cfg.hydro with this signature whenever P.hydro.enabled is true.
% The water-landing phase replaces it with hull-box buoyancy (per-box
% displaced volume -> restoring force and moment), Fossen linear+quadratic
% damping and added mass. Keep the signature; only the body changes.
%
% Frames: pos_ned in NED (z down, water surface at P.hydro.water_z_ned),
% v_b body velocity FRD, quat NED->body, pqr body rates. Outputs in FRD about
% the CG.

    F = zeros(3, 1);
    M = zeros(3, 1);
end
