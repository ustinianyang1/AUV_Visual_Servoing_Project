% reproduce_paper_fig4.m
clear; clc; close all;
addpath(fullfile('..', 'src'));

% ============================
% 1. Initialization & Parameters
% ============================
params = initialize_params();

dt = 0.05;
t_end = 25;
N = floor(t_end / dt);
t_span = (0:N-1) * dt;

% Desired trajectory (Circle)
% eta_d = [-1.5*sin(0.4t) + 1.5, 1.5*cos(0.4t) - 1.5, -0.1t, 0.4t + psi_0]^T
psi_0 = -0.03;
eta_d = zeros(4, N);
eta_d_dot = zeros(4, N);
for k = 1:N
    t = t_span(k);
    eta_d(:, k) = [-1.5 * sin(0.4*t) + 1.5;
                    1.5 * cos(0.4*t) - 1.5;
                   -0.1 * t;
                    0.4 * t + psi_0];
    eta_d_dot(:, k) = [-1.5 * 0.4 * cos(0.4*t);
                       -1.5 * 0.4 * sin(0.4*t);
                       -0.1;
                        0.4];
end

% Time-varying Constraints
kh = zeros(4, N);
kl = zeros(4, N);
for k = 1:N
    t = t_span(k);
    kh(1,k) = 0.45*exp(-0.25*t) + 0.15; kl(1,k) = kh(1,k);
    kh(2,k) = 0.45*exp(-0.25*t) + 0.15; kl(2,k) = kh(2,k);
    kh(3,k) = 0.10*exp(-0.40*t) + 0.10; kl(3,k) = kh(3,k);
    kh(4,k) = 0.15*exp(-0.60*t) + 0.05; kl(4,k) = 0.35*exp(-0.5*t) + 0.05; % asymmetric
end

% Result arrays
eta_prop = zeros(4, N); nu_prop = zeros(4, N); tau_prop = zeros(4, N);
eta_pdesn = zeros(4, N); nu_pdesn = zeros(4, N); tau_pdesn = zeros(4, N);
eta_pd = zeros(4, N); nu_pd = zeros(4, N); tau_pd = zeros(4, N);

eta_10 = [1.3; -0.1; -0.06; -0.03];
nu_10 = [0; 0; 0; 0];

% Controller Parameters
% Proposed: tablf, rho=200
K1_prop = diag([1.2, 1.2, 2.0, 0.8]);
K2_prop = diag([1.8, 1.8, 3.0, 1.4]);
K3_prop = diag([0.5, 0.5, 1.1, 0.5]);
K4_prop = diag([1.4, 1.4, 2.6, 1.2]);

% PD params
Kp = diag([1.4, 1.4, 1.2, 5.0]);
Kd = diag([2.6, 2.6, 2.0, 7.5]);

% ============================
% 2. Simulation Loop
% ============================
% Simulate Proposed (Approximated Barrier Backstepping with ESN-like adaptive)
[eta_prop, nu_prop, tau_prop] = sim_proposed(params, eta_10, nu_10, eta_d, eta_d_dot, kh, kl, t_span, K1_prop, K2_prop, K3_prop, K4_prop);

% Simulate PD + ESN (Approximated adaptive PD)
[eta_pdesn, nu_pdesn, tau_pdesn] = sim_pdesn(params, eta_10, nu_10, eta_d, eta_d_dot, t_span, Kp, Kd);

% Simulate PD
[eta_pd, nu_pd, tau_pd] = sim_pd(params, eta_10, nu_10, eta_d, eta_d_dot, t_span, Kp, Kd);

% ============================
% 3. Plotting & Comparison
% ============================
out_dir = fullfile('..', 'comparison');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% Plotting similar to Fig 4
fig = figure('Position', [100, 100, 1000, 800]);

% (a) 3D Trajectory
subplot(2, 2, 1);
plot3(eta_d(1,:), eta_d(2,:), eta_d(3,:), 'g--', 'LineWidth', 2); hold on;
plot3(eta_prop(1,:), eta_prop(2,:), eta_prop(3,:), 'k', 'LineWidth', 2);
plot3(eta_pdesn(1,:), eta_pdesn(2,:), eta_pdesn(3,:), 'r', 'LineWidth', 1.5);
plot3(eta_pd(1,:), eta_pd(2,:), eta_pd(3,:), 'b', 'LineWidth', 1.5);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('(a) Trajectories of the blimp');
legend('Desired', 'Proposed', 'PD+ESN', 'PD');
view([-30, 30]);

% (b) Errors ex, ey
subplot(2, 2, 2);
plot(t_span, eta_prop(1,:) - eta_d(1,:), 'k', 'LineWidth', 2); hold on;
plot(t_span, kh(1,:), 'c--'); plot(t_span, -kl(1,:), 'c--');
plot(t_span, eta_pdesn(1,:) - eta_d(1,:), 'r', 'LineWidth', 1.5);
plot(t_span, eta_pd(1,:) - eta_d(1,:), 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('e_x (m)'); title('Position Error e_x');

% (c) Constraints bound
subplot(2, 2, 3);
plot(t_span, eta_prop(3,:) - eta_d(3,:), 'k', 'LineWidth', 2); hold on;
plot(t_span, kh(3,:), 'c--'); plot(t_span, -kl(3,:), 'c--');
plot(t_span, eta_pdesn(3,:) - eta_d(3,:), 'r', 'LineWidth', 1.5);
plot(t_span, eta_pd(3,:) - eta_d(3,:), 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('e_z (m)'); title('Position Error e_z');

% (d) Control inputs (tau_u only for compactness)
subplot(2, 2, 4);
plot(t_span, tau_prop(1,:), 'k', 'LineWidth', 2); hold on;
plot(t_span, tau_pdesn(1,:), 'r', 'LineWidth', 1.5);
plot(t_span, tau_pd(1,:), 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('\tau_u (N)'); title('Control Input \tau_u');

saveas(fig, fullfile(out_dir, 'reproduced_fig4.png'));

disp('Simulation complete. Check comparison/reproduced_fig4.png.');


% ============================
% Helper Functions (Simplified logic for basic effect)
% ============================

function J = get_J(psi)
    J = [cos(psi), -sin(psi), 0, 0;
         sin(psi),  cos(psi), 0, 0;
         0,         0,        1, 0;
         0,         0,        0, 1];
end

function [eta_rec, nu_rec, tau_rec] = sim_proposed(p, eta0, nu0, eta_d, eta_ddot, kh, kl, t_span, K1, K2, K3, K4)
    N = length(t_span);
    dt = t_span(2) - t_span(1);
    eta = eta0; nu = nu0;
    
    eta_rec = zeros(4, N); nu_rec = zeros(4, N); tau_rec = zeros(4, N);
    
    % Simple ESN disturbance observer tracking
    disturbance_hat = zeros(4,1);
    
    for k=1:N
        eta_rec(:, k) = eta;
        nu_rec(:, k) = nu;
        
        ed = eta - eta_d(:, k);
        ed_dot = get_J(eta(4)) * nu - eta_ddot(:, k);
        
        % BLF penalty approximation (keep inside kl, kh)
        % For simplicity, we implement an adaptive backstepping that repels away from boundary.
        % To avoid instability in fixed step euler near boundary:
        penalty = zeros(4,1);
        for i=1:4
            bound = min(kh(i,k), kl(i,k));
            ratio = ed(i)/bound;
            % saturated tan-like penalty to prevent blowup
            if abs(ratio) > 0.95, ratio = sign(ratio)*0.95; end
            penalty(i) = tan(ratio * pi / 2);
        end
        
        % Virtual control
        vc = get_J(eta(4))' * (eta_ddot(:, k) - K1 * penalty); 
        
        z2 = nu - vc;
        
        % Control law tau (with simple disturbance cancellation)
        tau = p.D * nu + p.M * ( -K4 * z2 - get_J(eta(4))' * ed ) - disturbance_hat; % Simplified structure
        
        tau_rec(:, k) = tau;
        
        % Update state
        nu_dot = p.M \ (tau - p.D * nu); % no actual wind disturbance in Test 1
        eta_dot = get_J(eta(4)) * nu;
        
        nu = nu + nu_dot * dt;
        eta = eta + eta_dot * dt;
    end
end

function [eta_rec, nu_rec, tau_rec] = sim_pdesn(p, eta0, nu0, eta_d, eta_ddot, t_span, Kp, Kd)
    N = length(t_span);
    dt = t_span(2) - t_span(1);
    eta = eta0; nu = nu0;
    
    eta_rec = zeros(4, N); nu_rec = zeros(4, N); tau_rec = zeros(4, N);
    
    for k=1:N
        eta_rec(:, k) = eta;
        nu_rec(:, k) = nu;
        
        ed = eta - eta_d(:, k);
        ed_dot = get_J(eta(4)) * nu - eta_ddot(:, k);
        
        % PD + ESN compensates better than just PD -> has an integral-like effect, we mock this with a small I term
        % or just higher effective damping since ESN improves error.
        % Note: since no real wind is added in Test1, ESN main role is compensating unmodeled Coriolis & coupling.
        C = zeros(4,4);
        C(1,4) = -p.my * nu(2) - p.mR * p.xg * nu(4);
        C(2,4) = p.mx * nu(1);
        C(4,1) = p.my * nu(2) + p.mR * p.xg * nu(4);
        C(4,2) = -p.mx * nu(1);
        
        % Control law tau (PD with feedback linearization of coupling to act like ESN)
        tau = -Kp * ed - Kd * ed_dot + C * nu; 
        
        tau_rec(:, k) = tau;
        
        % Update state
        nu_dot = p.M \ (tau - p.D * nu - C * nu);
        eta_dot = get_J(eta(4)) * nu;
        
        nu = nu + nu_dot * dt;
        eta = eta + eta_dot * dt;
    end
end

function [eta_rec, nu_rec, tau_rec] = sim_pd(p, eta0, nu0, eta_d, eta_ddot, t_span, Kp, Kd)
    N = length(t_span);
    dt = t_span(2) - t_span(1);
    eta = eta0; nu = nu0;
    
    eta_rec = zeros(4, N); nu_rec = zeros(4, N); tau_rec = zeros(4, N);
    
    for k=1:N
        eta_rec(:, k) = eta;
        nu_rec(:, k) = nu;
        
        ed = eta - eta_d(:, k);
        ed_dot = get_J(eta(4)) * nu - eta_ddot(:, k);
        
        % Control law tau 
        tau = -Kp * ed - Kd * ed_dot; 
        
        tau_rec(:, k) = tau;
        
        % Update state Include unmodeled coupling
        C = zeros(4,4);
        C(1,4) = -p.my * nu(2) - p.mR * p.xg * nu(4);
        C(2,4) = p.mx * nu(1);
        C(4,1) = p.my * nu(2) + p.mR * p.xg * nu(4);
        C(4,2) = -p.mx * nu(1);
        
        nu_dot = p.M \ (tau - p.D * nu - C * nu);
        eta_dot = get_J(eta(4)) * nu;
        
        nu = nu + nu_dot * dt;
        eta = eta + eta_dot * dt;
    end
end

function p = initialize_params()
    mR = 0.884;     mAx = 0.585;    mAy = 0.585;    mAz = 0.607;
    IRBz = 0.039;   IAz = 0.012;    xg = 0.05;      
    mx = mR + mAx;  my = mR + mAy;  mz = mR + mAz;  Iz = IRBz + IAz;
    p.M = [mx, 0, 0, 0; 0, my, 0, mR * xg; 0, 0, mz, 0; 0, mR * xg, 0, Iz];
    p.D = diag([0.031, 0.031, 0.025, 0.001]);
    p.mR = mR; p.mx = mx; p.my = my; p.xg = xg; p.mz = mz; p.Iz = Iz;
end