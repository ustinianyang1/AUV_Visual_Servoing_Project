function [d_hat_x1, d_hat_x2, d_W_out, Phi] = esn_observer(x1, hat_x1, hat_x2, u_esn, Phi_delay, W_out_hat, W_in, W_d, M0, J, tau, K1, K2, rho, kw)
% ESN_OBSERVER Implements the Echo State Network based State Observer
%
% Inputs:
%   x1: measured position vector [x, y, z, psi]^T
%   hat_x1: estimated position vector
%   hat_x2: estimated velocity vector
%   u_esn: ESN input vector [x1(t); x1(t-td); x1(t-2td); tau(t)] (16x1)
%   Phi_delay: ESN reservoir state at t - td (8x1)
%   W_out_hat: current ESN output weights (8x4)
%   W_in: input weights (8x16)
%   W_d: reservoir internal weights (8x8)
%   M0: Inverse of or just nominal Inertia Matrix (4x4)
%   J: Rotation matrix (4x4)
%   tau: current control input (4x1)
%   K1, K2: Observer gain matrices (4x4)
%   rho, kw: Adaptation rate and leakage gain for W_out
%
% Outputs:
%   d_hat_x1: Time derivative of estimated position
%   d_hat_x2: Time derivative of estimated velocity
%   d_W_out: Time derivative of output weights
%   Phi: Current ESN reservoir state

    % 1. Update ESN Reservoir State Phi
    % Activation function: Gaussian function sigma(x) = exp(-x.^2)
    internal_activation = W_in * u_esn + W_d * Phi_delay;
    Phi = exp(-(internal_activation.^2));

    % 2. Observer Error
    tilde_x1 = hat_x1 - x1;

    % 3. State Observer Dynamics
    % d_hat_x1 = -K1 * tilde_x1 + J * hat_x2
    d_hat_x1 = -K1 * tilde_x1 + J * hat_x2;
    
    % g0 is nominal restoring force - assumed zero or compensated beforehand
    g0 = zeros(4,1); 
    
    % d_hat_x2 = -M0^{-1}*K2*J^T*tilde_x1 + M0^{-1}[tau - W_out_hat^T*Phi - g0]
    invM0 = inv(M0);
    d_hat_x2 = -invM0 * K2 * J' * tilde_x1 + invM0 * (tau - W_out_hat' * Phi - g0);

    % 4. Adaptive Law for Output Weights W_out_hat
    % d_W_out_hat = -rho * ( Phi * tilde_x1^T * J + kw * W_out_hat )
    d_W_out = -rho * ( Phi * tilde_x1' * J + kw * W_out_hat );

end
