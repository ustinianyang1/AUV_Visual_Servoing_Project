function [tau, nu_c, Lambda, hat_z2, num_term] = tablf_controller(x1, x1d, dx1d, d2x1d, hat_x2, nu_c_prev, dt, W_out_hat, Phi, J, M0, K4, kh, kl, d_kh, d_kl, K3i)
% TABLF_CONTROLLER Evaluates the Tangent Asymmetric Barrier Lyapunov Controller
%   Implements the asymmetric boundary control and calculates the actual thrust tau.
%
% Inputs:
%   x1     : Current position
%   x1d    : Desired position
%   dx1d   : Desired velocity
%   d2x1d  : Desired acceleration
%   hat_x2 : Estimated velocity (from ESN)
%   nu_c_prev : Previous virtual control (to estimate derivative)
%   dt     : Time step size
%   W_out_hat : ESN output weights (8x4)
%   Phi    : ESN state reservoir (8x1)
%   J      : Rotation matrix
%   M0     : Nominal Inertia matrix
%   K4     : Robust gain for backstepping
%   kh, kl : Upper and lower time-varying bounds for x1-x1d
%   d_kh, d_kl : Time derivatives of the bounds
%   K3i    : Diagonal terms of K3 gain matrix
%
% Outputs:
%   tau    : Actual control input [Fx, Fy, Fz, Tz]'
%   nu_c   : Virtual control
%   Lambda : Intermediate TABLF variable
%   hat_z2 : Observer velocity error
%   num_term : Numerical derivative of nu_c

    z1 = x1 - x1d;
    
    % Initialize Lambda
    Lambda = zeros(4, 1);
    Psi = zeros(4, 1); % Default Lyapunov associated gradient term

    for i = 1:4
        z = z1(i);
        
        q = double(z > 0);
        
        khi = kh(i);
        kli = kl(i);
        d_khi = d_kh(i);
        d_kli = d_kl(i);
        
        % Ensure bounds aren't somehow zero explicitly
        if khi < 1e-4, khi = 1e-4; end
        if kli < 1e-4, kli = 1e-4; end
        
        % Constrain z strictly inside bounds to prevent singularity during numerical integration
        max_ratio = 0.90; % Saturate ratio to prevent numeric blowup in explicit Euler
        if z >= khi * max_ratio, z = khi * max_ratio; end
        if z <= -kli * max_ratio, z = -kli * max_ratio; end
        
        % 1. Evaluate Term 1 for Lambda
        % num1 = (1-q) * kli^2 * sin(pi*z^2 / (2*kli^2)) + q * khi^2 * sin(pi*z^2 / (2*khi^2))
        num1 = (1-q) * kli^2 * sin(pi * z^2 / (2 * kli^2)) + q * khi^2 * sin(pi * z^2 / (2 * khi^2));
        
        % Use Taylor expansion / l'hopital around z=0 to prevent divide-by-zero
        if abs(z) < 1e-5
            % limit of num1 / z as z->0 is actually 0 since sin(z^2) ~ z^2.
            % Specifically: sin(pi z^2 / 2 k^2) ~ pi z^2 / 2 k^2.
            % So num1 ~ k^2 * (pi z^2 / 2 k^2) = (pi/2) z^2.
            % And num1 / (2*pi*z) ~ ((pi/2)*z^2) / (2*pi*z) = z / 4.
            term1 = -K3i(i) * (z / 4);
        else
            term1 = (-K3i(i) * num1) / (2 * pi * z);
        end
        
        % 2. Evaluate Term 2 for Lambda
        term2 = ( (1-q)*(d_kli/kli) + q*(d_khi/khi) ) * z;
        
        Lambda(i) = term1 + term2;
        
        % Evaluate Psi as the actual derivative of the TABLF!
        % V_i = k_i^2 / pi * tan(pi z_i^2 / 2 k_i^2)
        % dV_i/dz_i = z_i * sec^2(pi z_i^2 / 2 k_i^2) = z_i * (1 + tan^2(pi z_i^2 / 2 k_i^2))
        theta = pi * z^2 / (2 * ((1-q)*kli^2 + q*khi^2));
        Psi(i) = z * (1 + tan(theta)^2);
    end

    % TABLF Virtual Control
    nu_c = J' * (Lambda + dx1d);
    
    % Simple Numerical Differentiation for nu_c_dot (robust version)
    dot_nu_c = (nu_c - nu_c_prev) / dt;

    % Estimated Velocity Tracking Error
    hat_z2 = hat_x2 - nu_c;

    % Nominal restoring force assumption
    g0 = zeros(4,1);

    % Robust Backstepping Control Law tau
    % tau = W_out_hat^T * Phi + g0 + M0 * dot_nu_c - J^T * Psi - K4 * hat_z2
    tau = W_out_hat' * Phi + g0 + M0 * dot_nu_c - J' * Psi - K4 * hat_z2;

end
