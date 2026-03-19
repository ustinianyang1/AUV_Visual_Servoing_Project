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

% 注入一个 3N, 2N, 1N 的微小三轴水流扰动 (避免理想化)
base_tau_str = 'tau_E = [3.0; -2.0; 1.0; 0; 0; 0];';
dynamicsChart.Script = regexprep(backup_code, 'tau_E\s*=\s*zeros\(6,1\);', base_tau_str);

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
% 第二部分：50次蒙特卡洛剧烈扰动仿真 (纯净无噪声版)
% =========================================================================
disp('----------------------------------------------------');
disp('开始执行 50 次带随机剧烈洋流扰动的蒙特卡洛仿真...');
set_param(selectorPath, 'Value', '1');

num_tests = 50;
tau_E1_range = [-95.0, 93.7];
tau_E2_range = [-56.2, 14.5];
tau_E3_range = [-6.4, 13.9];

target_xv = 120; target_xu = 160; target_s1 = 27;
test_xv = zeros(num_tests, 1); test_xu = zeros(num_tests, 1);
mae_xv = zeros(num_tests, 1); mae_xu = zeros(num_tests, 1); mae_s1 = zeros(num_tests, 1);

try
    for i = 1:num_tests
        tau_E1 = tau_E1_range(1) + rand() * (tau_E1_range(2) - tau_E1_range(1));
        tau_E2 = tau_E2_range(1) + rand() * (tau_E2_range(2) - tau_E2_range(1));
        tau_E3 = tau_E3_range(1) + rand() * (tau_E3_range(2) - tau_E3_range(1));
        
        new_tau_str = sprintf('tau_E = [%.4f; %.4f; %.4f; 0; 0; 0];', tau_E1, tau_E2, tau_E3);
        dynamicsChart.Script = regexprep(backup_code, 'tau_E\s*=\s*zeros\(6,1\);', new_tau_str);
        
        fprintf('正在运行第 %d/%d 次蒙特卡洛剧烈干扰测试...\n', i, num_tests);
        out_mc = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
        
        eta_mc = squeeze(out_mc.out_eta);
        if size(eta_mc, 2) ~= 6 && size(eta_mc, 1) == 6; eta_mc = eta_mc'; end
        eta_end = eta_mc(end, :)';
        
        % 基础视觉映射 (严谨投影逻辑)
        x = eta_end(1); y = eta_end(2); z = eta_end(3); psi = eta_end(6);
        Z = 5.0 - z; if Z < 0.1; Z = 0.1; end
        
        err_x = x - (-0.04); err_y = y - 3.52;
        body_err_x = cos(psi) * err_x + sin(psi) * err_y;
        body_err_y = -sin(psi) * err_x + cos(psi) * err_y;
        
        % 去除人工测量噪声，只保留真实的系统稳态输出
        xv = 120 - 200.0 * (body_err_x / Z);
        xu = 160 - 200.0 * (body_err_y / Z);
        s1 = 27.0 * (1.55 / Z)^2;
        
        test_xv(i) = xv; test_xu(i) = xu;
        mae_xv(i) = abs(xv - target_xv); mae_xu(i) = abs(xu - target_xu); mae_s1(i) = abs(s1 - target_s1);
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
% 带透明度绘制，即使高度重合也能看出是叠加的散点
scatter(test_xv, test_xu, 50, rand(num_tests,1), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.5); hold on; grid on;
plot(target_xv, target_xu, 'rs', 'MarkerSize', 12, 'LineWidth', 2);
viscircles([target_xv, target_xu], 5, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
title('(a) Real Distribution diagram of test values', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Pixel x_v'); ylabel('Pixel x_u');
legend({'Test Points', 'Target Point'}, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);
% 限定坐标轴范围，避免全是完美 0 误差时图像缩放比例奇怪
axis([target_xv-10, target_xv+10, target_xu-10, target_xu+10]); 
colormap('parula'); colorbar;

subplot(1,2,2);
boxplot([mae_xv, mae_xu, mae_s1], 'Labels', {'x_v', 'x_u', 's_1'}, 'Widths', 0.5); grid on;
title('(b) Real Visual feature error distribution (MAE)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('MAE Value'); set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

exportgraphics(fig2, 'fig2.jpg', 'Resolution', 300);
disp('所有图表生成完毕！');