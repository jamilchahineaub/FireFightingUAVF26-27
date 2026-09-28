function q = quat_utils()
%QUAT_UTILS  Quaternion helpers as a struct of function handles.
%   q = quat_utils();
%   R = q.R_b2n(quat)          body -> NED rotation matrix (3x3)
%   R = q.R_n2b(quat)          NED  -> body
%   quat = q.from_euler(phi, theta, psi)
%   [phi, theta, psi] = q.to_euler(quat)
%   qdot = q.deriv(quat, omega)  d(quat)/dt for body rates omega = [p q r]
%
% Convention: quat = [q0 q1 q2 q3], scalar first, unit norm, representing the
% rotation from NED to body (Beard & McLain, Appendix B).

    q.R_b2n     = @R_b2n;
    q.R_n2b     = @(quat) R_b2n(quat)';
    q.from_euler = @from_euler;
    q.to_euler  = @to_euler;
    q.deriv     = @deriv;
    q.normalize = @(quat) quat / norm(quat);
end

function R = R_b2n(quat)
    e0 = quat(1); e1 = quat(2); e2 = quat(3); e3 = quat(4);
    R = [e1^2+e0^2-e2^2-e3^2,  2*(e1*e2-e3*e0),      2*(e1*e3+e2*e0);
         2*(e1*e2+e3*e0),      e2^2+e0^2-e1^2-e3^2,  2*(e2*e3-e1*e0);
         2*(e1*e3-e2*e0),      2*(e2*e3+e1*e0),      e3^2+e0^2-e1^2-e2^2];
end

function quat = from_euler(phi, theta, psi)
    cph = cos(phi/2); sph = sin(phi/2);
    cth = cos(theta/2); sth = sin(theta/2);
    cps = cos(psi/2); sps = sin(psi/2);
    quat = [cps*cth*cph + sps*sth*sph;
            cps*cth*sph - sps*sth*cph;
            cps*sth*cph + sps*cth*sph;
            sps*cth*cph - cps*sth*sph];
end

function [phi, theta, psi] = to_euler(quat)
    e0 = quat(1); e1 = quat(2); e2 = quat(3); e3 = quat(4);
    phi   = atan2(2*(e0*e1 + e2*e3), e0^2 + e3^2 - e1^2 - e2^2);
    s = 2*(e0*e2 - e1*e3);
    s = max(-1, min(1, s));
    theta = asin(s);
    psi   = atan2(2*(e0*e3 + e1*e2), e0^2 + e1^2 - e2^2 - e3^2);
end

function qd = deriv(quat, omega)
    p = omega(1); q = omega(2); r = omega(3);
    Om = [0 -p -q -r;
          p  0  r -q;
          q -r  0  p;
          r  q -p  0];
    lambda = 1000;                         % norm-correction gain
    qd = 0.5 * Om * quat(:) + lambda * (1 - quat(:)' * quat(:)) * quat(:);
end
