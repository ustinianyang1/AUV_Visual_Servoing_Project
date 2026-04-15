% test_closed_loop_control.m
clear; clc; close all;
addpath(fullfile('..', 'src'));

disp('Initializing TABLF and ESN Simulation...');

% ============================
% 1. Parameters & Setup
% ============================
params = blimp_params_init();

% Time Config for integration and ESN
dt = 0.001; % Use finer step 1ms to stabilize bare TABLF without magic limiters
T_end = 80;
N = floor(T_end / dt);
t_span = (0:N-1) * dt;

% ESN parameters 
n_neurons = 8;
t_d = 0.01; % Paper delay 10ms
delay_steps = ceil(t_d / dt); 
if delay_steps == 0, delay_steps = 1; end

% Initialize Random Weights
rng(42); % reproducibility
W_in = (rand(n_neurons, 16) * 2 - 1);
W_d = (rand(n_neurons, n_neurons) * 2 - 1);

% Ensure spectral radius of W_d < 1
sr = max(abs(eig(W_d)));
W_d = 0.95 * (W_d / sr);

rho = 200;
kw = 0.01;

% Controller Gains (TIE 2024 parameters)
K1 = diag([1.2, 1.2, 2.0, 0.8]);
K2 = diag([1.8, 1.8, 3.0, 1.4]);
K3 = diag([0.5, 0.5, 1.1, 0.5]);
K4 = diag([1.4, 1.4, 2.6, 1.2]);

% Initial conditions
x10 = [1.3; -0.1; -0.06; -0.03]; 
x20 = [0; 0; 0; 0];
% Real state
x1 = zeros(4, N); x1(:,1) = x10;
x2 = zeros(4, N); x2(:,1) = x20;
% Estimated state
hat_x1 = zeros(4, N); hat_x1(:,1) = x10;
hat_x2 = zeros(4, N); hat_x2(:,1) = x20;
hat_W_out = zeros(8, 4, N); 
Phi_history = zeros(8, N);

% Vectors for desired trajectory and boundaries
x1_d = zeros(4, N);
d_x1_d = zeros(4, N);
d2_x1_d = zeros(4, N);
tau_c = zeros(4, N);
nu_c_history = zeros(4, N);
dot_nu_c_history = zeros(4, N);

kh = zeros(4, N);
kl = zeros(4, N);
d_kh = zeros(4, N);
d_kl = zeros(4, N);

% Populate desired trajectory (Circle) and time-varying constraint bounds
for k = 1:N
    t = t_span(k);
    
    % Circle trajectory
    x1_d(:, k) = [-1.5*sin(0.4*t) + 1.5;
                  1.5*cos(0.4*t) - 1.5;
                 -0.1*t;
                  0.4*t - 0.03];
    
    d_x1_d(:, k) = [-1.5*0.4*cos(0.4*t);
                    -1.5*0.4*sin(0.4*t);
                    -0.1;
                     0.4];
                 
    d2_x1_d(:, k) = [1.5*0.4^2*sin(0.4*t);
                    -1.5*0.4^2*cos(0.4*t);
                     0;
                     0];
                 
    % Time varying constraints
    % Ex. 1, 2, 3 have symmetric bounds, 4 is asymmetric
    kh(1,k) = 0.45*exp(-0.25*t) + 0.15; kl(1,k) = kh(1,k);
    kh(2,k) = kh(1,k); kl(2,k) = kl(1,k);
    
    kh(3,k) = 0.10*exp(-0.40*t) + 0.10; kl(3,k) = kh(3,k);
    
    kh(4,k) = 0.15*exp(-0.60*t) + 0.05; 
    kl(4,k) = 0.35*exp(-0.50*t) + 0.05;
    
    d_kh(1,k) = -0.25*0.45*exp(-0.25*t); d_kl(1,k) = d_kh(1,k);
    d_kh(2,k) = d_kh(1,k); d_kl(2,k) = d_kl(1,k);
    
    d_kh(3,k) = -0.40*0.10*exp(-0.40*t); d_kl(3,k) = d_kh(3,k);
    
    d_kh(4,k) = -0.60*0.15*exp(-0.60*t); 
    d_kl(4,k) = -0.50*0.35*exp(-0.50*t);
end

% ============================
% 2. Main Simulation Loop
% ============================
M0 = params.M;

for k = 1:N-1
    t = t_span(k);
    
    % Rotation matrix J(psi)
    psi = x1(4, k);
    J = [cos(psi), -sin(psi), 0, 0;
         sin(psi),  cos(psi), 0, 0;
         0,         0,        1, 0;
         0,         0,        0, 1];
         
    % Identify ESN inputs u_esn(t)
    u_esn = zeros(16, 1);
    u_esn(1:4) = x1(:, k);
    
    idx_t1 = max(1, k - delay_steps);
    u_esn(5:8) = x1(:, idx_t1);
    
    idx_t2 = max(1, k - 2*delay_steps);
    u_esn(9:12) = x1(:, idx_t2);
    
    % Get previous ESN reservoir Phi and delayed tau
    if k == 1
        Phi_prev = zeros(8, 1);
        tau_prev = zeros(4, 1);
        % Prevent the derivative spike by calculating nu_c_0 exactly like tablf_controller does
        [~, nu_c_0, ~, ~, dot_nu_c_0] = tablf_controller( ...
            x1(:,k), x1_d(:,k), d_x1_d(:,k), d2_x1_d(:,k), ...
            hat_x2(:,k), zeros(4,1), zeros(4,1), dt, hat_W_out(:,:,k), zeros(8,1), ...
            J, M0, K4, kh(:,k), kl(:,k), d_kh(:,k), d_kl(:,k), diag(K3));
        nu_c_prev = nu_c_0;
        dot_nu_c_prev = dot_nu_c_0;
    else
        Phi_prev = Phi_history(:, max(1, k - delay_steps));
        tau_prev = tau_c(:, k-1);
        nu_c_prev = nu_c_history(:, k-1);
        dot_nu_c_prev = dot_nu_c_history(:, k-1);
    end
    u_esn(13:16) = tau_prev;
    
    % 1. Observer Step
    [d_hat_x1, d_hat_x2, d_W_out, Phi_current] = esn_observer(...
        x1(:,k), hat_x1(:,k), hat_x2(:,k), u_esn, Phi_prev, hat_W_out(:,:,k),...
        W_in, W_d, M0, J, tau_prev, K1, K2, rho, kw);
    
    Phi_history(:, k) = Phi_current;
    
    % Integrate ESN terms
    hat_x1(:, k+1) = hat_x1(:, k) + d_hat_x1 * dt;
    hat_x2(:, k+1) = hat_x2(:, k) + d_hat_x2 * dt;
    hat_W_out(:,:, k+1) = hat_W_out(:,:, k) + d_W_out * dt;
    
    % 2. Controller Step (TABLF)
    [tau, nu_c, ~, ~, dot_nu_c_current] = tablf_controller(...
        x1(:,k), x1_d(:,k), d_x1_d(:,k), d2_x1_d(:,k), ...
        hat_x2(:,k), nu_c_prev, dot_nu_c_prev, dt, hat_W_out(:,:,k), Phi_current, ...
        J, M0, K4, kh(:,k), kl(:,k), d_kh(:,k), d_kl(:,k), diag(K3));
        
    tau_c(:, k) = tau;
    nu_c_history(:, k) = nu_c;
    dot_nu_c_history(:, k) = dot_nu_c_current;
    if k<10
        fprintf('k=%d, x2(4)=%.4f, hat_x2(4)=%.4f\n', k, x2(4,k), hat_x2(4,k));
    end
    
    % --- NaN/Inf CHECK ---
    vars_to_check = {d_hat_x1, d_hat_x2, d_W_out, Phi_current, tau, nu_c};
    var_names = {'d_hat_x1', 'd_hat_x2', 'd_W_out', 'Phi_current', 'tau', 'nu_c'};
    for vi = 1:length(vars_to_check)
        if any(isnan(vars_to_check{vi}), 'all') || any(isinf(vars_to_check{vi}), 'all')
            fprintf('Warning: NaN/Inf detected in %s at step k=%d, t=%.3f\n', var_names{vi}, k, t);
            divergence_step = k;
        end
    end
    if exist('divergence_step', 'var') && divergence_step > 0
        break; % exit loop on divergence
    end

    % 3. Blimp Actual Dynamics Update (Plant) using RK4 to prevent BLF saturation explosion
    wind_disturbance = 0; % Set to const logic or zeros for standard
    tau_total = tau_c(:,k) + wind_disturbance;
    
    f_dx1 = @(psi_ang, vel) [cos(psi_ang), -sin(psi_ang), 0, 0; 
                             sin(psi_ang),  cos(psi_ang), 0, 0; 
                             0,             0,            1, 0; 
                             0,             0,            0, 1] * vel;
                         
    C_mat_f = @(vel) [0, 0, 0, -params.my*vel(2) - params.mR*params.xg*vel(4); 
                      0, 0, 0,  params.mx*vel(1); 
                      0, 0, 0,  0; 
                      params.my*vel(2) + params.mR*params.xg*vel(4), -params.mx*vel(1), 0, 0];
                  
    f_dx2 = @(vel) params.M \ (tau_total - C_mat_f(vel) * vel - params.D * vel);

    % RK4 Steps
    k1_x1 = f_dx1(x1(4,k), x2(:,k));
    k1_x2 = f_dx2(x2(:,k));
    
    k2_x1 = f_dx1(x1(4,k) + 0.5*dt*k1_x1(4), x2(:,k) + 0.5*dt*k1_x2);
    k2_x2 = f_dx2(x2(:,k) + 0.5*dt*k1_x2);
    
    k3_x1 = f_dx1(x1(4,k) + 0.5*dt*k2_x1(4), x2(:,k) + 0.5*dt*k2_x2);
    k3_x2 = f_dx2(x2(:,k) + 0.5*dt*k2_x2);
    
    k4_x1 = f_dx1(x1(4,k) + dt*k3_x1(4), x2(:,k) + dt*k3_x2);
    k4_x2 = f_dx2(x2(:,k) + dt*k3_x2);

    x1(:, k+1) = x1(:, k) + (dt/6) * (k1_x1 + 2*k2_x1 + 2*k3_x1 + k4_x1);
    x2(:, k+1) = x2(:, k) + (dt/6) * (k1_x2 + 2*k2_x2 + 2*k3_x2 + k4_x2);
end

disp('Simulation Logic complete.');

if exist('divergence_step', 'var') && divergence_step > 0
    warning('Simulation terminated early due to divergence at step %d', divergence_step);
    % truncate variables for plotting
    k_end = divergence_step;
    t_span = t_span(1:k_end);
    x1 = x1(:, 1:k_end);
    x2 = x2(:, 1:k_end);
    hat_x1 = hat_x1(:, 1:k_end);
    hat_x2 = hat_x2(:, 1:k_end);
    tau_c = tau_c(:, 1:k_end);
    x1_d = x1_d(:, 1:k_end);
    kh = kh(:, 1:k_end);
    kl = kl(:, 1:k_end);
end

% Save step-by-step parameters to a separate folder array
% 3. Calculate ESN Disturbance Estimates
hat_tau_D = zeros(4, N);
for k = 1:N
    hat_tau_D(:, k) = hat_W_out(:,:,k)' * Phi_history(:,k);
end

  % Calculate specifically requested velocity/control/disturbance alias tracking arrays
  e_v_vec = x2 - nu_c_history;
  e_v = e_v_vec(2, :);
  tau_v = tau_c(2, :);
  d_v = hat_tau_D(2, :);

  out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'data');
  if ~exist(out_dir, 'dir')
      mkdir(out_dir);
  end

  % Parameter Bounds based on paper
  bounds = struct();
  bounds.t_span = [0, 100];
  bounds.x1 = [-12, 42]; % [x, y, z, psi]
  bounds.x2 = [-15, 15]; % velocities
  bounds.hat_x1 = [-12, 42];
  bounds.hat_x2 = [-50, 50];
  bounds.tau_c = [-350, 350]; % thrust N
  bounds.x1_d = [-12, 42];
  bounds.hat_tau_D = [-4000, 4000];
  bounds.kh = [0, 2];
  bounds.kl = [0, 2];
  bounds.e_v = [-2, 2];
  bounds.tau_v = [-350, 350];
  bounds.d_v = [-4000, 4000];

  vars_to_save = {'t_span', 'x1', 'x2', 'hat_x1', 'hat_x2', 'tau_c', 'x1_d', 'hat_tau_D', 'kh', 'kl', 'e_v', 'tau_v', 'd_v'};
  
  fprintf('\n--- Parameter Range Validation & Saving ---\n');
  for i = 1:length(vars_to_save)
      v_name = vars_to_save{i};
      v_data = eval(v_name);
      
      % Range verification
      if isfield(bounds, v_name)
          b = bounds.(v_name);
          v_min = min(v_data(:));
          v_max = max(v_data(:));
          if v_min < b(1) || v_max > b(2)
              fprintf('! WARNING: %s exceeds bound [%.1f, %.1f] limit. Found [%.2f, %.2f]\n', v_name, b(1), b(2), v_min, v_max);
          else
              fprintf('OK: %s within bounds [%.1f, %.1f]. Actual span: [%.2f, %.2f]\n', v_name, b(1), b(2), v_min, v_max);
          end
      end
      
      % Save individually to comparison/
      file_path = fullfile(out_dir, [v_name, '.mat']);
      save(file_path, v_name);
  end

fig = figure('Position', [100, 100, 800, 900]);
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(t_span, x1(1,:) - x1_d(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t_span, x1(2,:) - x1_d(2,:), 'g', 'LineWidth', 1.5);
plot(t_span, x1(3,:) - x1_d(3,:), 'b', 'LineWidth', 1.5);
plot(t_span, x1(4,:) - x1_d(4,:), 'm', 'LineWidth', 1.5);
plot(t_span, kh(1,:), 'k--', 'LineWidth', 1.5);
plot(t_span, -kl(1,:), 'k--', 'LineWidth', 1.5);
grid on; ylabel('Tracking Errors');
legend('e_x', 'e_y', 'e_z', 'e_\psi', 'k_h', '-k_l');
title('(a) Tracking errors');
xlim([0, T_end]);

% (b) Control Inputs
nexttile;
plot(t_span, tau_c(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t_span, tau_c(2,:), 'g', 'LineWidth', 1.5);
plot(t_span, tau_c(3,:), 'b', 'LineWidth', 1.5);
plot(t_span, tau_c(4,:), 'm', 'LineWidth', 1.5);
grid on; ylabel('Control Inputs');
legend('\tau_u', '\tau_v', '\tau_w', '\tau_\psi', 'NumColumns', 4);
title('(b) Control inputs');
xlim([0, T_end]);

% (c) Disturbance Estimates
nexttile;
plot(t_span, hat_tau_D(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t_span, hat_tau_D(2,:), 'g', 'LineWidth', 1.5);
plot(t_span, hat_tau_D(3,:), 'b', 'LineWidth', 1.5);
plot(t_span, hat_tau_D(4,:), 'm', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Disturbance Estimates');
legend('~d_u', '~d_v', '~d_w', '~d_\psi', 'NumColumns', 4);
title('(c) Disturbance estimates');
xlim([0, T_end]);

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'comparison');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
saveas(fig, fullfile(out_dir, 'simulation_results.png'));
disp('Plots successfully saved to comparison/simulation_results.png');



function p = blimp_params_init()
    mR = 0.884;     mAx = 0.585;    mAy = 0.585;    mAz = 0.607;
    IRBz = 0.039;   IAz = 0.012;    xg = 0.05;      
    mx = mR + mAx;  my = mR + mAy;  mz = mR + mAz;  Iz = IRBz + IAz;
    p.M = [mx, 0, 0, 0; 0, my, 0, mR * xg; 0, 0, mz, 0; 0, mR * xg, 0, Iz];
    p.D = diag([0.031, 0.031, 0.025, 0.001]);
    p.mR = mR; p.mx = mx; p.my = my; p.xg = xg; p.mz = mz; p.Iz = Iz;
end