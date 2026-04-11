function nu_dot = blimp_dynamics(nu, tau, tau_d, params)
% BLIMP_DYNAMICS - Non-linear dynamics for the 4-DOF indoor blimp
%
%   nu:     Body-frame velocities [u, v, w, r]^T
%   tau:    Control input forces/moments [Tu, Tv, Tw, Tr]^T
%   tau_d:  External disturbance forces   [du, dv, dw, dr]^T
%   params: Structure loaded by blimp_params.m containing mass/damping
%
%   Returns:
%       nu_dot: Accelerations [u_dot, v_dot, w_dot, r_dot]^T in body frame.

    u = nu(1);
    v = nu(2);
    % w = nu(3); % z-velocity
    r = nu(4);
    
    % Access physical constants
    mx = params.mx;
    my = params.my;
    % mz = params.mz;
    mR = params.mR;
    xg = params.xg;
    
    M = params.M;
    D = params.D;
    
    % Compute the Coriolis & Centripetal Matrix C(nu)
    % C(nu) accounts for nonlinear velocity coupling in cross products.
    C = zeros(4, 4);
    % For a standard 4DOF formulation without roll and pitch:
    % (Based on standard AUV planar dynamics equations)
    C(1, 4) = -my * v - mR * xg * r;
    C(2, 4) =  mx * u;
    C(4, 1) =  my * v + mR * xg * r;
    C(4, 2) = -mx * u;
    
    % Equation: M*nu_dot + C(nu)*nu + D*nu = tau + tau_d
    % Isolate nu_dot for the ODE solver:
    % nu_dot = M \ (tau + tau_d - C*nu - D*nu)
    
    rhs = tau + tau_d - C * nu - D * nu;
    
    % Solve for acceleration (nu_dot)
    nu_dot = M \ rhs;
    
end
