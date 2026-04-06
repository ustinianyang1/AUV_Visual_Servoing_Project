% =========================================================================
% 室内飞艇轨迹跟踪控制 - 完整复现运行脚本
% 包含：参数初始化、运行 Simulink 模型、指标计算、图像生成与保存
% =========================================================================
clear; clc; close all;

%% 1. 参数初始化 (严格遵循论文，并应用防奇异与维度修正)
m_R = 0.884; m_Ax = 0.585; m_Ay = 0.585; m_Az = 0.607;
I_RBz = 0.039; I_Az = 0.012; x_g = 0.05;
D_vx = 0.031; D_vy = 0.031; D_vz = 0.025; D_wz = 0.001; % [修正] D_wx 改为 D_wz
g = 9.8; f_G = m_R * g; f_B = f_G;

% 惯性与阻尼矩阵
M0 = diag([m_R+m_Ax, m_R+m_Ay, m_R+m_Az, I_RBz+I_Az]);
M0(2,4) = m_R * x_g; M0(4,2) = m_R * x_g;
D0 = diag([D_vx, D_vy, D_vz, D_wz]);
g0 = [0; 0; f_G - f_B; 0];

% ESN 观测器参数
rho = 200; k_w = 0.01; t_d = 0.01;
K1_prime = diag([1.2, 1.2, 2.0, 0.8]);
K2 = diag([1.8, 1.8, 3.0, 1.4]);
n_reservoir = 8;
W_in = rand(n_reservoir, 16) * 0.2 - 0.1; % [修正] 保证与16维输入匹配
W_d = rand(n_reservoir, n_reservoir) * 0.2 - 0.1;

% 控制器参数
K3 = diag([0.5, 0.5, 1.1, 0.5]);
K4 = diag([1.4, 1.4, 2.6, 1.2]);

% 初始状态 (Exp1-Test1 圆形轨迹)
x1_0 = [1.3; -0.1; -0.06; -0.03]; 
x2_0 = [0; 0; 0; 0];
x2_hat_0 = [0; 0; 0; 0];
W_out_0 = zeros(n_reservoir, 4);

disp('参数初始化完成，开始运行 Simulink 仿真...');

%% 2. 运行 Simulink 模型
% 必须确保同一目录下已构建好 Blimp_Model.slx
simTime = 200;
simOut = sim('Blimp_Model.slx', 'StopTime', num2str(simTime));

disp('仿真完成，正在计算指标与生成图像...');

%% 3. 数据提取与指标计算
t = simOut.tout;
x1 = simOut.x1;       % 实际位置 [N x 4]
x1d = simOut.x1d;     % 期望位置 [N x 4]
tau = simOut.tau;     % 控制输入 [N x 4]

% 误差计算
e = x1 - x1d;
e_dis = sqrt(e(:,1).^2 + e(:,2).^2 + e(:,3).^2);
e_psi = abs(e(:,4));

% 计算并输出复现指标
max_e_dis = max(e_dis);
rms_e_dis = sqrt(mean(e_dis.^2));
max_e_psi = max(e_psi);
rms_e_psi = sqrt(mean(e_psi.^2));

fprintf('\n=== Exp1-Test1 复现指标 ===\n');
fprintf('Max e_dis: %.4f m\n', max_e_dis);
fprintf('Max e_psi: %.4f rad\n', max_e_psi);
fprintf('RMS e_dis: %.4f m\n', rms_e_dis);
fprintf('RMS e_psi: %.4f rad\n', rms_e_psi);

%% 4. 论文同款图像绘制与保存
% 图1: 3D 轨迹图 (对应论文 Fig 4a)
fig1 = figure('Name', '3D Trajectory', 'Position', [100, 100, 600, 400]);
plot3(x1(:,1), x1(:,2), x1(:,3), 'k-', 'LineWidth', 1.5); hold on;
plot3(x1d(:,1), x1d(:,2), x1d(:,3), 'k--', 'LineWidth', 1.5);
grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Proposed method', 'Desired trajectories');
view(3);
saveas(fig1, 'Fig4a_Trajectory.png');

% 图2: 跟踪误差及约束边界 (对应论文 Fig 4b)
k_l1 = 0.5*exp(-0.4*t) + 0.1; k_h1 = 0.4*exp(-0.4*t) + 0.2;
k_l2 = 0.5*exp(-0.4*t) + 0.1; k_h2 = 0.4*exp(-0.4*t) + 0.2;
k_l3 = 0.3*exp(-0.4*t) + 0.15; k_h3 = 0.3*exp(-0.7*t) + 0.1;
k_l4 = 0.1*exp(-0.3*t) + 0.015; k_h4 = 0.1*exp(-0.2*t) + 0.015;

fig2 = figure('Name', 'Tracking Errors', 'Position', [750, 100, 600, 800]);
titles = {'e_x (m)', 'e_y (m)', 'e_z (m)', 'e_{\psi} (rad)'};
err_data = [e(:,1), e(:,2), e(:,3), e(:,4)];
kl_data = [k_l1, k_l2, k_l3, k_l4];
kh_data = [k_h1, k_h2, k_h3, k_h4];

for i = 1:4
    subplot(4, 1, i);
    plot(t, err_data(:,i), 'k-', 'LineWidth', 1.2); hold on;
    plot(t, kh_data(:,i), 'm-', 'LineWidth', 1);
    plot(t, -kl_data(:,i), 'm-', 'LineWidth', 1);
    ylabel(titles{i}); grid on;
    xlim([0 200]);
    if i == 1
        legend('Proposed method', 'Constraints', 'Location', 'best');
    end
end
xlabel('Time (sec)');
saveas(fig2, 'Fig4b_Errors.png');

% 图3: 控制输入 (对应论文 Fig 4c)
fig3 = figure('Name', 'Control Inputs', 'Position', [1400, 100, 600, 800]);
tau_titles = {'f_x (N)', 'f_y (N)', 'f_z (N)', '\tau_{\psi} (N\cdot m)'};
for i = 1:4
    subplot(4, 1, i);
    plot(t, tau(:,i), 'b-', 'LineWidth', 1.2);
    ylabel(tau_titles{i}); grid on;
    xlim([0 200]);
end
xlabel('Time (sec)');
saveas(fig3, 'Fig4c_ControlInputs.png');

disp('图像生成完毕并已保存为 .png 文件。');