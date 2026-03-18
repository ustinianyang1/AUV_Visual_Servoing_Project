% =========================================================================
% AUV visual servoing figure generation script
% 已添加：自动保存高清图表 (fig1.jpg, fig2.jpg, fig3.jpg)
% =========================================================================
clear; clc; close all;
modelName = 'AUV_Visual_Servoing_System';

% 加载模型
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

% =========================================================================
% 第一部分：基准仿真
% =========================================================================
disp('正在自动运行单次基准仿真，提取动态响应数据...');
out = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
disp('基准仿真完毕！正在生成图 1 和 图 3...');

t = out.tout;
eta = squeeze(out.out_eta); if size(eta, 2) ~= 6 && size(eta, 1) == 6; eta = eta'; end
v = squeeze(out.out_v); if size(v, 2) ~= 6 && size(v, 1) == 6; v = v'; end
tau = squeeze(out.out_tau); if size(tau, 2) ~= 6 && size(tau, 1) == 6; tau = tau'; end

% --- 图1：位姿跟踪 ---
fig1 = figure('Name', 'Figure 1: Pose and Attitude Tracking', 'Color', 'w', 'Position', [50, 100, 800, 600]);
subplot(2,1,1);
plot(t, eta(:,1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, eta(:,2), 'b', 'LineWidth', 1.5);
plot(t, eta(:,3), 'k', 'LineWidth', 1.5);
title('Positions (x, y, z)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman'); ylabel('Position (m)', 'FontName', 'Times New Roman');
legend({'x', 'y', 'z'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

subplot(2,1,2);
plot(t, eta(:,4), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, eta(:,5), 'b', 'LineWidth', 1.5);
plot(t, eta(:,6), 'k', 'LineWidth', 1.5);
title('Euler Angles (\phi, \theta, \psi)', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Time (s)', 'FontName', 'Times New Roman'); ylabel('Angle (rad)', 'FontName', 'Times New Roman');
legend({'\phi', '\theta', '\psi'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% 保存图 1
exportgraphics(fig1, 'fig1.jpg', 'Resolution', 300);
disp('已保存 fig1.jpg');

% --- 图3：速度与控制输入 ---
fig3 = figure('Name', 'Figure 3: Velocities and Control Inputs', 'Color', 'w', 'Position', [150, 200, 1000, 700]);
subplot(2,2,1);
plot(t, v(:,1), 'r', 'LineWidth', 1.2); hold on; grid on; plot(t, v(:,2), 'b', 'LineWidth', 1.2); plot(t, v(:,3), 'k', 'LineWidth', 1.2);
title('Linear Velocities (u, v, w)', 'FontName', 'Times New Roman', 'FontSize', 12); xlabel('Time (s)', 'FontName', 'Times New Roman'); ylabel('Velocity (m/s)', 'FontName', 'Times New Roman'); legend({'u', 'v', 'w'}, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman');

subplot(2,2,2);
plot(t, v(:,4), 'r', 'LineWidth', 1.2); hold on; grid on; plot(t, v(:,5), 'b', 'LineWidth', 1.2); plot(t, v(:,6), 'k', 'LineWidth', 1.2);
title('Angular Velocities (p, q, r)', 'FontName', 'Times New Roman', 'FontSize', 12); xlabel('Time (s)', 'FontName', 'Times New Roman'); ylabel('Velocity (rad/s)', 'FontName', 'Times New Roman'); legend({'p', 'q', 'r'}, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman');

subplot(2,2,3);
plot(t, tau(:,1), 'r', 'LineWidth', 1.2); hold on; grid on; plot(t, tau(:,2), 'b', 'LineWidth', 1.2); plot(t, tau(:,3), 'k', 'LineWidth', 1.2);
title('Control Forces (\tau_u, \tau_v, \tau_w)', 'FontName', 'Times New Roman', 'FontSize', 12); xlabel('Time (s)', 'FontName', 'Times New Roman'); ylabel('Force (N)', 'FontName', 'Times New Roman'); legend({'\tau_u', '\tau_v', '\tau_w'}, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman');

subplot(2,2,4);
plot(t, tau(:,4), 'r', 'LineWidth', 1.2); hold on; grid on; plot(t, tau(:,5), 'b', 'LineWidth', 1.2); plot(t, tau(:,6), 'k', 'LineWidth', 1.2);
title('Control Torques (\tau_p, \tau_q, \tau_r)', 'FontName', 'Times New Roman', 'FontSize', 12); xlabel('Time (s)', 'FontName', 'Times New Roman'); ylabel('Torque (N\cdot m)', 'FontName', 'Times New Roman'); legend({'\tau_p', '\tau_q', '\tau_r'}, 'Location', 'best'); set(gca, 'FontName', 'Times New Roman');

% 保存图 3
exportgraphics(fig3, 'fig3.jpg', 'Resolution', 300);
disp('已保存 fig3.jpg');

% =========================================================================
% 第二部分：50次蒙特卡洛扰动仿真
% =========================================================================
disp('----------------------------------------------------');
disp('开始执行真正的 50 次带随机洋流扰动的蒙特卡洛仿真...');

% 查找动力学模块
rt = sfroot;
charts = rt.find('-isa', 'Stateflow.EMChart');
targetChart = [];
for k = 1:length(charts)
    if contains(charts(k).Path, modelName) && contains(charts(k).Script, 'AUV_Dynamics')
        targetChart = charts(k);
        break;
    end
end
if isempty(targetChart)
    error('在 Simulink 模型中未找到 AUV_Dynamics 模块，请确认模型已打开且拼写正确。');
end

% 备份模块代码
backup_code = targetChart.Script;
num_tests = 50;
tau_E1_range = [-95.0, 93.7];
tau_E2_range = [-56.2, 14.5];
tau_E3_range = [-6.4, 13.9];
target_xv = 120; target_xu = 160; target_s1 = 27;
test_xv = zeros(num_tests, 1); test_xu = zeros(num_tests, 1);
mae_xv = zeros(num_tests, 1); mae_xu = zeros(num_tests, 1); mae_s1 = zeros(num_tests, 1);

try
    for i = 1:num_tests
        % 生成扰动
        tau_E1 = tau_E1_range(1) + rand() * (tau_E1_range(2) - tau_E1_range(1));
        tau_E2 = tau_E2_range(1) + rand() * (tau_E2_range(2) - tau_E2_range(1));
        tau_E3 = tau_E3_range(1) + rand() * (tau_E3_range(2) - tau_E3_range(1));
        
        % 注入扰动
        new_tau_str = sprintf('tau_E = [%.4f; %.4f; %.4f; 0; 0; 0];', tau_E1, tau_E2, tau_E3);
        modified_code = regexprep(backup_code, 'tau_E\s*=\s*zeros\(6,1\);', new_tau_str);
        
        targetChart.Script = modified_code;
        
        % 运行仿真
        fprintf('正在运行第 %d/%d 次洋流干扰测试...\n', i, num_tests);
        out_mc = sim(modelName, 'StopTime', '120', 'FixedStep', '0.01');
        
        % 提取稳态数据
        eta_mc = squeeze(out_mc.out_eta);
        if size(eta_mc, 2) ~= 6 && size(eta_mc, 1) == 6; eta_mc = eta_mc'; end
        eta_end = eta_mc(end, :)';
        
        % 视觉映射
        x = eta_end(1); y = eta_end(2); z = eta_end(3);
        phi = eta_end(4); theta = eta_end(5); psi = eta_end(6);
        Z = 5.0 - z; if Z < 0.1; Z = 0.1; end
        err_x = x - (-0.04); err_y = y - 3.52;
        body_err_x = cos(psi) * err_x + sin(psi) * err_y;
        body_err_y = -sin(psi) * err_x + cos(psi) * err_y;
        xv = 120 - 200.0 * (body_err_x / Z);
        xu = 160 - 200.0 * (body_err_y / Z);
        s1 = 27.0 * (1.55 / Z)^2;
        
        test_xv(i) = xv;
        test_xu(i) = xu;
        mae_xv(i) = abs(xv - target_xv);
        mae_xu(i) = abs(xu - target_xu);
        mae_s1(i) = abs(s1 - target_s1);
    end
    
    % 恢复模块代码
    targetChart.Script = backup_code;
    
catch ME
    % 异常时确保恢复
    disp('仿真中断！正在还原 AUV_Dynamics 模块代码...');
    if ~isempty(targetChart)
        targetChart.Script = backup_code;
    end
    rethrow(ME);
end

disp('50次仿真全部完成！正在生成图 2...');

% --- 图2：统计性能 ---
fig2 = figure('Name', 'Figure 2: Real Statistical Performance (50 runs)', 'Color', 'w', 'Position', [100, 150, 900, 450]);
subplot(1,2,1);
scatter(test_xv, test_xu, 30, rand(num_tests,1), 'filled', 'MarkerEdgeColor', 'k'); hold on; grid on;
plot(target_xv, target_xu, 'rs', 'MarkerSize', 10, 'LineWidth', 2);
viscircles([target_xv, target_xu], 5, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
title('(a) Real Distribution diagram of test values', 'FontName', 'Times New Roman', 'FontSize', 12);
xlabel('Pixel x_v', 'FontName', 'Times New Roman');
ylabel('Pixel x_u', 'FontName', 'Times New Roman');
legend({'Test Points', 'Target Point'}, 'Location', 'best', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);
colormap('parula'); colorbar;

subplot(1,2,2);
boxplot([mae_xv, mae_xu, mae_s1], 'Labels', {'x_v', 'x_u', 's_1'}, 'Widths', 0.5);
grid on;
title('(b) Real Visual feature error distribution (MAE)', 'FontName', 'Times New Roman', 'FontSize', 12);
ylabel('MAE Value', 'FontName', 'Times New Roman');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% 保存图 2
exportgraphics(fig2, 'fig2.jpg', 'Resolution', 300);
disp('已保存 fig2.jpg');
disp('所有 3 张论文核心图表均已生成并保存！');