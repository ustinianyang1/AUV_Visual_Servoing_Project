% test_blimp_model.m
% Validates the 4-DOF Open Loop Dynamics structure of the Blimp.
% Implements an ode45 continuous time simulation of the system tracking an
% open-loop force field input.

clc; clear; close all;

% Get the current script's directory to set robust paths
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'src'));

disp('--- Blimp ODE45 Open-Loop Simulation Test ---');

% 1) Initialize workspace variables and parameters structure
params_struct = blimp_params_init();  % Avoid cluttering, we'll create a local helper.

% Initial conditions: x = [eta; nu]
% [X(1:4)]: eta (pose)
% [X(5:8)]: nu  (vels)
X0 = zeros(8,1);

% Define simulation time config
t_span = [0 15]; % Running for 15 seconds.

%% Integrate ODE
% Options
opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-5);
disp('Commencing ode45 integration...');
[t, X_out] = ode45(@(t, x) blimp_ode(t, x, params_struct), t_span, X0, opts);
disp('Integration complete.');

%% Plot Results
out_dir = fullfile(script_dir, '..', 'comparison');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

fig = figure('Name','Indoor Blimp Open-Loop Trajectory');
subplot(2,2,1);
plot3(X_out(:,1), X_out(:,2), X_out(:,3), 'LineWidth', 2, 'Color','b');
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Position Trace');
view(45, 30);

subplot(2,2,2);
plot(X_out(:,1), X_out(:,2), 'LineWidth', 2);
title('XY Plane Path Trace');
xlabel('X (m)'); ylabel('Y (m)');
grid on; axis equal;

subplot(2,2,3);
plot(t, X_out(:,4)*180/pi, 'r', 'LineWidth', 2);
title('Yaw Angle Trace (\psi)');
xlabel('Time (s)'); ylabel('Angle (deg)');
grid on;

subplot(2,2,4);
plot(t, X_out(:,5:7), 'LineWidth', 1.5);
title('Velocities (u, v, w)');
legend('u','v','w');
xlabel('Time (s)'); ylabel('Vel (m/s)');
grid on;

% Save the figure
saveas(fig, fullfile(out_dir, 'open_loop_trajectory.png'));

%==============================================%
% Functions and ODE Handles                    %
%==============================================%
function dX_dt = blimp_ode(t, X, params)
    % Extract arrays from state X
    eta = X(1:4);
    nu  = X(5:8);
    
    % Open-Loop Control configuration (Constant forward, small turning mom)
    % tau = [Force_u, Force_v, Force_w, Moment_r]^T
    tau = [0.2; 0; 0.1; 0.05]; 
    
    % Assumed zero un-modeled structural disturbances
    tau_d = zeros(4,1); 
    
    % Call Equations from other functions/scripts:
    eta_dot = blimp_kinematics(eta, nu);
    nu_dot  = blimp_dynamics(nu, tau, tau_d, params);
    
    % Concatenate derivatives vector
    dX_dt = [eta_dot; nu_dot];
end

function p = blimp_params_init()
    %% Mass properties
    mR = 0.884;     % Rigid body mass (kg)
    mAx = 0.585;    % Added mass in x (kg)
    mAy = 0.585;    % Added mass in y (kg)
    mAz = 0.607;    % Added mass in z (kg)

    IRBz = 0.039;   % Body moment of inertia around z (kg*m^2)
    IAz = 0.012;    % Added moment of inertia around z (kg*m^2)

    xg = 0.05;      
    % Total mass variables
    mx = mR + mAx;
    my = mR + mAy;
    mz = mR + mAz;
    Iz = IRBz + IAz;
    
    M_mat = [mx, 0, 0, 0;
             0, my, 0, mR * xg;
             0, 0, mz, 0;
             0, mR * xg, 0, Iz];

    D_mat = diag([0.031, 0.031, 0.025, 0.001]);
    
    p.mR = mR; p.xg = xg; p.mx = mx; p.my = my; p.mz = mz; p.Iz = Iz;
    p.M = M_mat;
    p.D = D_mat;
end
