% blimp_params.m
% This script initializes the physical and hydrodynamic parameters of the indoor blimp
% based on "Adaptive Output Feedback Trajectory Tracking Control of an Indoor Blimp"

%% Mass properties
mR = 0.884;     % Rigid body mass (kg)
mAx = 0.585;    % Added mass in x (kg)
mAy = 0.585;    % Added mass in y (kg)
mAz = 0.607;    % Added mass in z (kg)

IRBz = 0.039;   % Body moment of inertia around z (kg*m^2)
IAz = 0.012;    % Added moment of inertia around z (kg*m^2)

xg = 0.05;      % Center of gravity offset in x-axis (m)

% Total mass variables
mx = mR + mAx;
my = mR + mAy;
mz = mR + mAz;
Iz = IRBz + IAz;

%% System Matrices
% Inertia matrix M (4x4)
% considering only (x, y, z, r) or (u, v, w, p) representing velocities
% Since x_g = 0.05, there is a coupling between v-axis and yaw r.
M_mat = [mx, 0, 0, 0;
         0, my, 0, mR * xg;
         0, 0, mz, 0;
         0, mR * xg, 0, Iz];

% Damping matrix D (4x4)
Dvx = 0.031;
Dvy = 0.031;
Dvz = 0.025;
Dwz = 0.001;

D_mat = diag([Dvx, Dvy, Dvz, Dwz]);

%% Pack into a structure for easy passing to ordinary differential equations
params.mR = mR;
params.xg = xg;
params.mx = mx;
params.my = my;
params.mz = mz;
params.Iz = Iz;
params.M = M_mat;
params.D = D_mat;

disp('Blimp parameters loaded successfully.');
