% AUV 视觉伺服控制仿真主入口
clc; clear; close all;

% --- 1. 参数初始化 ---
m = 98.0; W = 961.38; B = 952.56; x_G = 0; y_G = 0; z_G = 0.05;
X_u_dot = 49.0; Y_v_dot = 49.0; Z_w_dot = 49.0; I_x = 8.0; I_y = 8.0; I_z = 8.0;
X_uu = -148.0; Y_vv = -148.0; Z_ww = -148.0; 
K_pp = -180.0; M_qq = -180.0; N_rr = -180.0;
K_p = -130.0; M_q = -130.0; N_r = -130.0;

M_mat = diag([m+X_u_dot, m+Y_v_dot, m+Z_w_dot, I_x, I_y, I_z]);
M_inv = inv(M_mat);

k1 = 2; k2 = 100; b1 = 50 * eye(6); k3 = diag([90, 90, 5, 1, 1, 1]); d_j = 2; num_rules = 17;
rng(0); q_j = -4 + 8 * rand(18, num_rules);

eta_0 = [-0.04; 3.52; 3.45; 0; 0; -1.87];
nu_0 = zeros(6,1);
xi_hat = [120; 160; 27; 0; 0; -1.87];
s1_0 = 6.75; phi_0 = zeros(num_rules, 6); e2_hat_0 = zeros(6,1);
tau_E_inject = zeros(6,1);

% --- 2. 运行仿真 ---
model_name = 'Adaptive_Fuzzy_HVS';
load_system(model_name);
set_param(model_name, 'Solver', 'ode15s', 'MaxStep', '0.01', 'StopTime', '120');
disp('正在运行仿真，请稍候...');
simOut = sim(model_name);
disp('仿真结束，正在绘制结果...');

% --- 3. 结果可视化 ---
Plot_Results(simOut);
