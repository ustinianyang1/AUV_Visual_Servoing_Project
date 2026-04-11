function eta_dot = blimp_kinematics(eta, nu)
% BLIMP_KINEMATICS - Computes the time derivative of the 4-DOF pose vector
% 
%   eta: Pose vector [x, y, z, psi]^T in inertial frame.
%   nu:  Velocity vector [u, v, w, r]^T in body-fixed frame.
%
%   Returns:
%       eta_dot: Resulting time derivative vector of eta.

    % Extract yaw angle
    psi = eta(4);

    % Rotation matrix J(eta) for 4-DOF planar-like flight + z (no roll/pitch)
    J = [cos(psi), -sin(psi), 0, 0;
         sin(psi),  cos(psi), 0, 0;
         0,         0,        1, 0;
         0,         0,        0, 1];

    % Convert body velocities to inertial coordinate rates
    eta_dot = J * nu;
end
