% =========================================================================
% Girona 500 AUV 视觉伺服控制系统：一键仿真与论文同款图表生成脚本
% 修正了 Simulink 数组维度过高导致无法转置的报错
% =========================================================================
clear; clc; close all;

modelName = 'AUV_Visual_Servoing_System';

disp('正在自动运行 Simulink 仿真，请稍候 (预计几秒钟)...');
% 强制运行仿真，并将数据输出捕获到 out 对象中
out = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
disp('仿真运行完毕！正在提取数据并绘制论文同款图表...');

% 1. 提取时间矩阵
t = out.tout;

% 提取位姿数据 (eta)，使用 squeeze 挤压掉多余的三维外壳
eta = squeeze(out.out_eta);
if size(eta, 2) ~= 6 && size(eta, 1) == 6; eta = eta'; end

% 提取速度数据 (v)
v = squeeze(out.out_v);
if size(v, 2) ~= 6 && size(v, 1) == 6; v = v'; end

% 提取推力/扭矩数据 (tau)
tau = squeeze(out.out_tau);
if size(tau, 2) ~= 6 && size(tau, 1) == 6; tau = tau'; end

% 提取视觉特征误差数据 (e1)
e1 = squeeze(out.out_e1);
if size(e1, 2) ~= 6 && size(e1, 1) == 6; e1 = e1'; end

% =========================================================================
% 图 1：位姿状态响应曲线 (对应论文 Figure 3 风格)
% =========================================================================
figure('Name', 'Figure 3: Pose Response', 'Color', 'w', 'Position', [100, 100, 800, 600]);

% X, Y, Z 位置
subplot(2,1,1);
plot(t, eta(:,1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, eta(:,2), 'b', 'LineWidth', 1.5);
plot(t, eta(:,3), 'k', 'LineWidth', 1.5);
title('Positions (x, y, z)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Position (m)', 'FontName', 'Times New Roman');
legend({'x', 'y', 'z'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% 姿态角 (Phi, Theta, Psi)
subplot(2,1,2);
plot(t, eta(:,4), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, eta(:,5), 'b', 'LineWidth', 1.5);
plot(t, eta(:,6), 'k', 'LineWidth', 1.5);
title('Euler Angles (\phi, \theta, \psi)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Angle (rad)', 'FontName', 'Times New Roman');
legend({'\phi', '\theta', '\psi'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% =========================================================================
% 图 2：速度状态响应曲线 (对应论文 Figure 4 风格)
% =========================================================================
figure('Name', 'Figure 4: Velocity Response', 'Color', 'w', 'Position', [150, 150, 800, 600]);

% 线速度
subplot(2,1,1);
plot(t, v(:,1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, v(:,2), 'b', 'LineWidth', 1.5);
plot(t, v(:,3), 'k', 'LineWidth', 1.5);
title('Linear Velocities (u, v, w)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Velocity (m/s)', 'FontName', 'Times New Roman');
legend({'u', 'v', 'w'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% 角速度
subplot(2,1,2);
plot(t, v(:,4), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, v(:,5), 'b', 'LineWidth', 1.5);
plot(t, v(:,6), 'k', 'LineWidth', 1.5);
title('Angular Velocities (p, q, r)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Angular Velocity (rad/s)', 'FontName', 'Times New Roman');
legend({'p', 'q', 'r'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% =========================================================================
% 图 3：控制输入扭矩/推力曲线 (对应论文 Figure 5 风格)
% =========================================================================
figure('Name', 'Figure 5: Control Inputs', 'Color', 'w', 'Position', [200, 200, 800, 600]);

% 力
subplot(2,1,1);
plot(t, tau(:,1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, tau(:,2), 'b', 'LineWidth', 1.5);
plot(t, tau(:,3), 'k', 'LineWidth', 1.5);
title('Control Forces (\tau_u, \tau_v, \tau_w)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Force (N)', 'FontName', 'Times New Roman');
legend({'\tau_u', '\tau_v', '\tau_w'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% 力矩
subplot(2,1,2);
plot(t, tau(:,4), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, tau(:,5), 'b', 'LineWidth', 1.5);
plot(t, tau(:,6), 'k', 'LineWidth', 1.5);
title('Control Torques (\tau_p, \tau_q, \tau_r)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Torque (N\cdot m)', 'FontName', 'Times New Roman');
legend({'\tau_p', '\tau_q', '\tau_r'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% =========================================================================
% 图 4：视觉特征误差收敛曲线 (对应论文视觉伺服核心收敛图)
% =========================================================================
figure('Name', 'Visual Feature Errors', 'Color', 'w', 'Position', [250, 250, 800, 400]);
plot(t, e1(:,1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, e1(:,2), 'b', 'LineWidth', 1.5);
plot(t, e1(:,3), 'k', 'LineWidth', 1.5);
title('Visual Feature Tracking Errors (e_1)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman');
ylabel('Error Value', 'FontName', 'Times New Roman');
legend({'e_{1,x_v}', 'e_{1,x_u}', 'e_{1,s}'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

disp('全部图表绘制完成！');