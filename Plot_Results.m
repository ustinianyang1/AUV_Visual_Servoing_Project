% Plot_Results.m 终极无脑兼容版 (完美适配 out_xxx 命名)
clc; close all;
disp('📊 正在提取数据并绘图...');

%% 1. 提取数据 (暴力+智能读取机制)
% --- 读取 out_time ---
if exist('simOut', 'var') && isprop(simOut, 'out_time')
    t = squeeze(simOut.out_time);
elseif exist('out', 'var') && isprop(out, 'out_time')
    t = squeeze(out.out_time);
elseif exist('out_time', 'var')
    t = squeeze(out_time);
else
    error('找不到 out_time！请检查 Simulink 时钟输出模块是否命名为 out_time 且格式为 Array。');
end

% --- 读取 out_eta ---
if exist('simOut', 'var') && isprop(simOut, 'out_eta')
    eta = squeeze(simOut.out_eta);
elseif exist('out', 'var') && isprop(out, 'out_eta')
    eta = squeeze(out.out_eta);
elseif exist('out_eta', 'var')
    eta = squeeze(out_eta);
else
    error('找不到 out_eta！');
end
if size(eta, 2) ~= 6; eta = eta'; end

% --- 读取 out_nu ---
if exist('simOut', 'var') && isprop(simOut, 'out_nu')
    v = squeeze(simOut.out_nu);
elseif exist('out', 'var') && isprop(out, 'out_nu')
    v = squeeze(out.out_nu);
elseif exist('out_nu', 'var')
    v = squeeze(out_nu);
else
    error('找不到 out_nu！');
end
if size(v, 2) ~= 6; v = v'; end

% --- 读取 out_tau ---
if exist('simOut', 'var') && isprop(simOut, 'out_tau')
    tau = squeeze(simOut.out_tau);
elseif exist('out', 'var') && isprop(out, 'out_tau')
    tau = squeeze(out.out_tau);
elseif exist('out_tau', 'var')
    tau = squeeze(out_tau);
else
    error('找不到 out_tau！');
end
if size(tau, 2) ~= 6; tau = tau'; end

% 目标位姿 (用于画黑色虚线参考线)
eta_d = [-0.04; 3.52; 3.45; 0; 0; -1.87];

%% 2. 绘制 Fig.3: 位姿响应
figure('Name', 'Fig.3: Pose Response', 'Position', [100, 100, 1000, 600]);
titles_eta = {'(a) x (m)', '(b) y (m)', '(c) z (m)', '(d) \phi (rad)', '(e) \theta (rad)', '(f) \psi (rad)'};
for i = 1:6
    subplot(2, 3, i);
    plot(t, eta(:, i), 'b', 'LineWidth', 1.5); hold on;
    % 对于前三项和最后一项画目标参考线，横滚和俯仰目标为0
    if i == 3 || i == 6
        plot(t, eta_d(i)*ones(size(t)), 'k--', 'LineWidth', 1);
    elseif i == 4 || i == 5
        plot(t, zeros(size(t)), 'k--', 'LineWidth', 1);
    end
    title(titles_eta{i}); xlabel('time (s)'); grid on; xlim([0, 120]);
end

%% 3. 绘制 Fig.4: 速度响应
figure('Name', 'Fig.4: Velocity Response', 'Position', [150, 150, 1000, 600]);
titles_v = {'(a) v_u (m/s)', '(b) v_v (m/s)', '(c) v_w (m/s)', '(d) v_p (rad/s)', '(e) v_q (rad/s)', '(f) v_r (rad/s)'};
for i = 1:6
    subplot(2, 3, i);
    plot(t, v(:, i), 'b', 'LineWidth', 1.5); hold on;
    plot(t, zeros(size(t)), 'k--', 'LineWidth', 1); % 速度目标稳态均为 0
    title(titles_v{i}); xlabel('time (s)'); grid on; xlim([0, 120]);
end

%% 4. 绘制 Fig.5: 控制量响应
figure('Name', 'Fig.5: Control Input', 'Position', [200, 200, 1000, 600]);
titles_tau = {'(a) \tau_1 (N)', '(b) \tau_2 (N)', '(c) \tau_3 (N)', '(d) \tau_4 (N\cdot m)', '(e) \tau_5 (N\cdot m)', '(f) \tau_6 (N\cdot m)'};
for i = 1:6
    subplot(2, 3, i);
    plot(t, tau(:, i), 'b', 'LineWidth', 1.5);
    title(titles_tau{i}); xlabel('time (s)'); grid on; xlim([0, 120]);
end

%% 5. 计算稳态误差 (对标论文 Tab.V-VI)
% 提取最后 10 秒 (110s - 120s) 作为稳态区间
idx_steady = find(t >= 110);
if ~isempty(idx_steady)
    MAE_eta = mean(abs(eta(idx_steady, :) - [eta(1,1:2), eta_d(3:6)']));
    MAE_v = mean(abs(v(idx_steady, :)));

    disp('======================================================');
    disp('📊 论文对标结果验证:');
    disp('【Tab.V 对标】位姿稳态绝对误差 [x, y, z, phi, theta, psi]:');
    fprintf('%.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', MAE_eta);
    disp('【Tab.VI 对标】速度稳态绝对误差 [u, v, w, p, q, r]:');
    fprintf('%.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', MAE_v);
    disp('======================================================');
else
    disp('⚠️ 警告：仿真时间不足 110 秒，无法计算稳态误差。');
end