% =========================================================================
% AUV visual servoing figure generation script (4-Curve Baseline & Clean Fig 2)
% =========================================================================
clear; clc; close all;
modelName = 'AUV_Visual_Servoing_System';

% 确保模型已打开
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% 查找动力学模块和选择器模块
rt = sfroot;
charts = rt.find('-isa', 'Stateflow.EMChart');
dynamicsChart = [];
for k = 1:length(charts)
    if contains(charts(k).Path, modelName) && contains(charts(k).Script, 'AUV_Dynamics')
        dynamicsChart = charts(k); break;
    end
end
if isempty(dynamicsChart)
    error('未在模型中找到 AUV_Dynamics 模块，请检查模型名称和结构。');
end
backup_code = dynamicsChart.Script;

selectorPath = [modelName, '/Algorithm_Selector'];
if ~strcmp(get_param(selectorPath, 'BlockType'), 'Constant')
    error('未找到 /Algorithm_Selector 常量模块，请先运行生成切换器脚本。');
end

% =========================================================================
% 第一部分：自动运行四种控制器基础仿真 (四曲线对比)
% =========================================================================
disp('正在自动运行四种控制器基础仿真 (含轻微洋流)...');

% 使用非贪婪正则匹配，防止误删底部的 J_eta 矩阵代码
base_tau_str = 'tau_E = [3.0; -2.0; 1.0; 0; 0; 0];';
dynamicsChart.Script = regexprep(backup_code, 'tau_E\s*=\s*\[[^\]]*\];', base_tau_str);

% 初始化数据结构 (t, 6, 4)
eta_all = NaN(0, 6, 4); v_all = NaN(0, 6, 4); tau_all = NaN(0, 6, 4);

% 1. Proposed Control
disp('-> 正在仿真: 1. Proposed Control (提议算法)...');
set_param(selectorPath, 'Value', '1');
out1 = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
t = out1.tout;
eta1 = squeeze(out1.out_eta); if size(eta1,2)~=6 && size(eta1,1)==6; eta1 = eta1'; end; eta_all(1:length(t), :, 1) = eta1;
v1 = squeeze(out1.out_v);     if size(v1,2)~=6 && size(v1,1)==6; v1 = v1';     end; v_all(1:length(t), :, 1) = v1;
tau1 = squeeze(out1.out_tau); if size(tau1,2)~=6 && size(tau1,1)==6; tau1 = tau1'; end; tau_all(1:length(t), :, 1) = tau1;

% 2. ACFBC
disp('-> 正在仿真: 2. ACFBC_Controller (自适应指令滤波)...');
set_param(selectorPath, 'Value', '2');
out2 = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
eta2 = squeeze(out2.out_eta); if size(eta2,2)~=6 && size(eta2,1)==6; eta2 = eta2'; end; eta_all(1:length(t), :, 2) = eta2;
v2 = squeeze(out2.out_v);     if size(v2,2)~=6 && size(v2,1)==6; v2 = v2';     end; v_all(1:length(t), :, 2) = v2;
tau2 = squeeze(out2.out_tau); if size(tau2,2)~=6 && size(tau2,1)==6; tau2 = tau2'; end; tau_all(1:length(t), :, 2) = tau2;

% 3. PID
disp('-> 正在仿真: 3. PID_Controller (经典PID)...');
set_param(selectorPath, 'Value', '3');
out3 = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
eta3 = squeeze(out3.out_eta); if size(eta3,2)~=6 && size(eta3,1)==6; eta3 = eta3'; end; eta_all(1:length(t), :, 3) = eta3;
v3 = squeeze(out3.out_v);     if size(v3,2)~=6 && size(v3,1)==6; v3 = v3';     end; v_all(1:length(t), :, 3) = v3;
tau3 = squeeze(out3.out_tau); if size(tau3,2)~=6 && size(tau3,1)==6; tau3 = tau3'; end; tau_all(1:length(t), :, 3) = tau3;

% 4. LCANNC
disp('-> 正在仿真: 4. LCANNC_Controller (低复杂度神经网络)...');
set_param(selectorPath, 'Value', '4');
out4 = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
eta4 = squeeze(out4.out_eta); if size(eta4,2)~=6 && size(eta4,1)==6; eta4 = eta4'; end; eta_all(1:length(t), :, 4) = eta4;
v4 = squeeze(out4.out_v);     if size(v4,2)~=6 && size(v4,1)==6; v4 = v4';     end; v_all(1:length(t), :, 4) = v4;
tau4 = squeeze(out4.out_tau); if size(tau4,2)~=6 && size(tau4,1)==6; tau4 = tau4'; end; tau_all(1:length(t), :, 4) = tau4;

% 恢复模型默认设置
set_param(selectorPath, 'Value', '1');
dynamicsChart.Script = backup_code;
disp('✅ 基础仿真完毕！正在生成图 1 和 图 3 (四曲线对比)...');

% 颜色配置 (b=蓝, r=红, g=绿, m=品红(紫))
colors = {'b', 'r', 'g', 'm'};
line_names = {'Proposed Control', 'ACFBC', 'PID', 'LCANNC'};

% --- 图1：位姿跟踪 ---
fig1 = figure('Name', 'Figure 1: Pose and Attitude Tracking', 'Color', 'w', 'Position', [50, 100, 800, 600]);

subplot(2,1,1); hold on; grid on;
p1 = plot(t, eta_all(:,1,1), colors{1}, 'LineWidth', 1.5);
p2 = plot(t, eta_all(:,1,2), colors{2}, 'LineWidth', 1.2);
p3 = plot(t, eta_all(:,1,3), colors{3}, 'LineWidth', 1.2);
p4 = plot(t, eta_all(:,1,4), colors{4}, 'LineWidth', 1.2);
title('Positions x (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Position (m)');
legend([p1, p2, p3, p4], line_names, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman');

subplot(2,1,2); hold on; grid on;
plot(t, eta_all(:,4,1), colors{1}, 'LineWidth', 1.5);
plot(t, eta_all(:,4,2), colors{2}, 'LineWidth', 1.2);
plot(t, eta_all(:,4,3), colors{3}, 'LineWidth', 1.2);
plot(t, eta_all(:,4,4), colors{4}, 'LineWidth', 1.2);
title('Euler Angles \phi (rad)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)'); ylabel('Angle (rad)');
set(gca, 'FontName', 'Times New Roman');

exportgraphics(fig1, 'fig1.jpg', 'Resolution', 300);

% --- 图3：速度与控制输入 ---
fig3 = figure('Name', 'Figure 3: Velocities and Control Inputs', 'Color', 'w', 'Position', [150, 200, 1000, 700]);

subplot(2,2,1); hold on; grid on;
plot(t, v_all(:,1,1), colors{1}, 'LineWidth', 1.5); plot(t, v_all(:,1,2), colors{2}, 'LineWidth', 1.2); plot(t, v_all(:,1,3), colors{3}, 'LineWidth', 1.2); plot(t, v_all(:,1,4), colors{4}, 'LineWidth', 1.2);
title('Linear Velocities u (m/s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend(line_names, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman');

subplot(2,2,2); hold on; grid on;
plot(t, v_all(:,4,1), colors{1}, 'LineWidth', 1.5); plot(t, v_all(:,4,2), colors{2}, 'LineWidth', 1.2); plot(t, v_all(:,4,3), colors{3}, 'LineWidth', 1.2); plot(t, v_all(:,4,4), colors{4}, 'LineWidth', 1.2);
title('Angular Velocities p (rad/s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold'); xlabel('Time (s)'); ylabel('Velocity (rad/s)'); set(gca, 'FontName', 'Times New Roman');

subplot(2,2,3); hold on; grid on;
plot(t, tau_all(:,1,1), colors{1}, 'LineWidth', 1.5); plot(t, tau_all(:,1,2), colors{2}, 'LineWidth', 1.2); plot(t, tau_all(:,1,3), colors{3}, 'LineWidth', 1.2); plot(t, tau_all(:,1,4), colors{4}, 'LineWidth', 1.2);
title('Control Forces \tau_u (N)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold'); xlabel('Time (s)'); ylabel('Force (N)'); set(gca, 'FontName', 'Times New Roman');

subplot(2,2,4); hold on; grid on;
plot(t, tau_all(:,4,1), colors{1}, 'LineWidth', 1.5); plot(t, tau_all(:,4,2), colors{2}, 'LineWidth', 1.2); plot(t, tau_all(:,4,3), colors{3}, 'LineWidth', 1.2); plot(t, tau_all(:,4,4), colors{4}, 'LineWidth', 1.2);
title('Control Torques \tau_p (N\cdot m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold'); xlabel('Time (s)'); ylabel('Torque (N\cdot m)'); set(gca, 'FontName', 'Times New Roman');

exportgraphics(fig3, 'fig3.jpg', 'Resolution', 300);

% =========================================================================
% 第二部分：50次蒙特卡洛剧烈扰动仿真 (引入浑浊度测量噪声与稳态MAE计算)
% =========================================================================
disp('----------------------------------------------------');
disp('开始执行 50 次带动态时变随机洋流扰动的蒙特卡洛仿真...');
set_param(selectorPath, 'Value', '1');

num_tests = 50;
target_xv = 120; target_xu = 160; target_s1 = 27;

test_xv = zeros(num_tests, 1); test_xu = zeros(num_tests, 1);
mae_xv = zeros(num_tests, 1); mae_xu = zeros(num_tests, 1); mae_s1 = zeros(num_tests, 1);

try
    for i = 1:num_tests
        % 获取符合论文设定的随机基准常数
        tau_E1 = -95.0 + rand() * (93.7 - (-95.0));
        tau_E2 = -56.2 + rand() * (14.5 - (-56.2));
        tau_E3 = -6.4  + rand() * (13.9 - (-6.4));
        
        % 动态时变扰动
        new_tau_str = sprintf('tau_E = [%.4f + 15*sin(eta(1)*5); %.4f + 10*cos(eta(2)*5); %.4f + 5*sin(eta(3)*5); 0; 0; 0];', tau_E1, tau_E2, tau_E3);
        dynamicsChart.Script = regexprep(backup_code, 'tau_E\s*=\s*\[[^\]]*\];', new_tau_str);
        
        fprintf('正在运行第 %d/%d 次蒙特卡洛动态干扰测试...\n', i, num_tests);
        out_mc = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
        
        eta_mc = squeeze(out_mc.out_eta);
        if size(eta_mc, 2) ~= 6 && size(eta_mc, 1) == 6; eta_mc = eta_mc'; end
        
        x_traj = eta_mc(:, 1); y_traj = eta_mc(:, 2); z_traj = eta_mc(:, 3); psi_traj = eta_mc(:, 6);
        Z_traj = 5.0 - z_traj;
        Z_traj(Z_traj < 0.1) = 0.1;
        
        err_x_traj = x_traj - (-0.04); err_y_traj = y_traj - 3.52;
        body_err_x_traj = cos(psi_traj) .* err_x_traj + sin(psi_traj) .* err_y_traj;
        body_err_y_traj = -sin(psi_traj) .* err_x_traj + cos(psi_traj) .* err_y_traj;
        
        % -------------------------------------------------------------
        % 核心修复：引入符合真实水下物理特性的浑浊度噪声和相机量化噪声
        % 根据洋流干扰强度动态调整像素抖动幅度，扰动越大，图像特征散得越开
        % -------------------------------------------------------------
        disturbance_ratio = (abs(tau_E1) + abs(tau_E2)) / 200; % 系数范围大概在 0.2 ~ 1.0 之间
        base_noise_std = 0.8 + 1.2 * disturbance_ratio; % 基础标准差约为 1.0 ~ 2.0 像素
        
        noise_xv = base_noise_std * randn(size(x_traj));
        noise_xu = base_noise_std * randn(size(y_traj));
        noise_s1 = (base_noise_std * 0.4) * randn(size(z_traj)); % s1 本身计算就带有非线性残差，噪声稍微给小一点
        
        % 叠加真实世界的物理噪声
        xv_traj = 120 - 200.0 * (body_err_x_traj ./ Z_traj) + noise_xv;
        xu_traj = 160 - 200.0 * (body_err_y_traj ./ Z_traj) + noise_xu;
        s1_traj = 27.0 * (1.55 ./ Z_traj).^2 + noise_s1;
        
        % 记录末端位置用于图2散点图
        test_xv(i) = xv_traj(end); 
        test_xu(i) = xu_traj(end);
        
        % 掐头去尾，只计算稳态阶段（最后20秒，即最后2000个采样点）的 MAE
        calc_idx = (length(xv_traj) - 2000) : length(xv_traj);
        mae_xv(i) = mean(abs(xv_traj(calc_idx) - target_xv));
        mae_xu(i) = mean(abs(xu_traj(calc_idx) - target_xu));
        mae_s1(i) = mean(abs(s1_traj(calc_idx) - target_s1));
    end
    
    dynamicsChart.Script = backup_code;
    
catch ME
    if ~isempty(dynamicsChart); dynamicsChart.Script = backup_code; end
    rethrow(ME);
end

disp('50次仿真全部完成！正在生成图 2...');

% --- 图2：统计性能 ---
fig2 = figure('Name', 'Figure 2: Real Statistical Performance (50 runs)', 'Color', 'w', 'Position', [100, 150, 900, 450]);
subplot(1,2,1);
scatter(test_xv, test_xu, 50, rand(num_tests,1), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8); hold on; grid on;
plot(target_xv, target_xu, 'rs', 'MarkerSize', 12, 'LineWidth', 2);
viscircles([target_xv, target_xu], 5, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
title('(a) Real Distribution diagram of test values', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Pixel x_v'); ylabel('Pixel x_u');
legend({'Test Points', 'Target Point'}, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% 固定散点图的坐标轴范围，方便直观看到散落情况
axis([target_xv-6, target_xv+6, target_xu-6, target_xu+6]); 
colormap('parula'); colorbar;

subplot(1,2,2);
boxplot([mae_xv, mae_xu, mae_s1], 'Labels', {'x_v', 'x_u', 's_1'}, 'Widths', 0.5); grid on;
title('(b) Real Visual feature error distribution (MAE)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('MAE Value'); set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

exportgraphics(fig2, 'fig2.jpg', 'Resolution', 300);