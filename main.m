% AUV 视觉伺服控制仿真主入口
clc; clear; close all;

% --- 1. 参数初始化 ---
m = 98.0; W = 961.38; B = 952.56; x_G = 0; y_G = 0; z_G = 0.05;
X_u_dot = 49.0; Y_v_dot = 49.0; Z_w_dot = 49.0; I_x = 8.0; I_y = 8.0; I_z = 8.0;
M_mat = diag([m+X_u_dot, m+Y_v_dot, m+Z_w_dot, I_x, I_y, I_z]);
M_inv = inv(M_mat);

% 控制增益微调以保证数值稳定性
<<<<<<< HEAD
k1 = 2; k2 = 100; b1 = 50 * eye(6); k3 = diag([90, 90, 5, 1, 1, 1]); d_j = 2; num_rules = 17;
=======
k1 = 2; k2 = 80; b1 = 50 * eye(6); k3 = diag([90, 90, 5, 1, 1, 1]); d_j = 2; num_rules = 17;
>>>>>>> 0451bd63f3f63a14b9dc926cca31be87d4aa8af4
rng(0); q_j = -4 + 8 * rand(18, num_rules);

eta_0 = [-0.04; 3.52; 3.45; 0; 0; -1.87];
nu_0 = zeros(6,1);
xi_hat = [120; 160; 27; 0; 0; -1.87];
s1_0 = 6.75; phi_0 = zeros(num_rules, 6); e2_hat_0 = zeros(6,1);
tau_E_inject = zeros(6,1);

% --- 2. 运行仿真 ---
model_name = 'Adaptive_Fuzzy_HVS';
load_system(model_name);
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.01');
set_param(model_name, 'StopTime', '120');
<<<<<<< HEAD
disp('采用 0.01s 定步长仿真，正在全力加速中...');
simOut = sim(model_name);
disp('仿真完成，准备出图！');
=======
disp('🚀 采用 0.01s 定步长仿真，正在全力加速中...');
simOut = sim(model_name);
disp('✅ 仿真完成，准备出图！');
>>>>>>> 0451bd63f3f63a14b9dc926cca31be87d4aa8af4

% --- 3. 绘图 ---
Plot_Results(simOut);
