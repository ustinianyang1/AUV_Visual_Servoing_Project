% =========================================================================
% 室内飞艇轨迹跟踪控制 - 最终修正版复现脚本
% 解决：变量未定义(t_full)、长度不匹配、参数重载
% =========================================================================
clear; clc; close all;

%% 1. 环境初始化
n_reservoir = 8;
W_in = rand(n_reservoir, 16) * 0.2 - 0.1; 
W_d = rand(n_reservoir, n_reservoir) * 0.2 - 0.1;
assignin('base', 'W_in', W_in);
assignin('base', 'W_d', W_d);

modelName = 'Blimp_Model';
if ~bdIsLoaded(modelName), open_system(modelName); end

% 强制关闭 To Workspace 限制 (兼容性修复)
toWsBlocks = find_system(modelName, 'BlockType', 'ToWorkspace');
for i = 1:length(toWsBlocks)
    try set_param(toWsBlocks{i}, 'LimitDataPoints', 'off'); catch; end
    try set_param(toWsBlocks{i}, 'MaxRows', 'inf'); catch; end
    set_param(toWsBlocks{i}, 'SaveFormat', 'Array');
end

%% 2. 运行仿真
disp('>>> 仿真正在运行，请稍候...');
simOut = sim(modelName, 'StopTime', '200');

%% 3. 核心修复：安全提取数据与长度对齐
% 优先从 simOut 提取时间轴，若不存在则尝试从 tout 提取
if isprop(simOut, 'tout')
    t_full = simOut.tout;
elseif exist('tout','var')
    t_full = tout;
else
    error('无法获取仿真时间向量，请检查模型配置。');
end

% 提取输出数据
x1 = simOut.get('x1');
x1d = simOut.get('x1d');
tau = simOut.get('tau');

% 自动计算最小共有长度，防止 plot 报错
data_len = min([length(t_full), size(x1,1), size(x1d,1), size(tau,1)]);

% 统一截断
t = t_full(1:data_len);
x1 = x1(1:data_len, :);
x1d = x1d(1:data_len, :);
tau = tau(1:data_len, :);

%% 4. 指标计算
e = x1 - x1d;
e_dis = sqrt(sum(e(:,1:3).^2, 2));
e_psi = abs(e(:,4));

fprintf('\n=== Exp1-Test1 复现指标 ===\n');
fprintf('Max位移误差: %.4f m (目标: 0.1448)\n', max(e_dis));
fprintf('Max偏航误差: %.4f rad (目标: 0.0322)\n', max(e_psi));
fprintf('RMS位移误差: %.4f m (目标: 0.0318)\n', sqrt(mean(e_dis.^2)));

%% 5. 绘制论文同款图像

% --- Fig 4a: 3D Trajectory ---
fig4a = figure('Color','w','Name','Fig 4a: 3D Trajectory');
plot3(x1(:,1), x1(:,2), x1(:,3), 'b-', 'LineWidth', 1.5); hold on; % 实际轨迹
plot3(x1d(:,1), x1d(:,2), x1d(:,3), 'r--', 'LineWidth', 1.2); % 期望轨迹
grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Proposed method', 'Desired trajectories', 'Location', 'best');
view(3);
saveas(fig4a, 'Fig4a_Trajectory.png');

% --- Fig 4b: Tracking Errors & TABLF Constraints ---
fig4b = figure('Color','w','Name','Fig 4b: Tracking Errors','Position',[100,100,600,800]);
titles = {'e_x (m)', 'e_y (m)', 'e_z (m)', 'e_{\psi} (rad)'};
for i = 1:4
    subplot(4,1,i);
    plot(t, e(:,i), 'k-', 'LineWidth', 1.2); hold on; % 误差曲线
    % 计算对应的 TABLF 时变边界
    if i==1, kl = 0.5*exp(-0.4*t)+0.1; kh = 0.4*exp(-0.4*t)+0.2;
    elseif i==2, kl = 0.5*exp(-0.4*t)+0.1; kh = 0.4*exp(-0.4*t)+0.2;
    elseif i==3, kl = 0.3*exp(-0.4*t)+0.15; kh = 0.3*exp(-0.7*t)+0.1;
    else, kl = 0.1*exp(-0.3*t)+0.015; kh = 0.1*exp(-0.2*t)+0.015;
    end
    plot(t, kh, 'm-', 'LineWidth', 1); % 上边界
    plot(t, -kl, 'm-', 'LineWidth', 1); % 下边界[cite: 2]
    ylabel(titles{i}); grid on; xlim([0 200]);
    if i == 1, legend('Proposed method', 'Constraints'); end
end
xlabel('Time (sec)');
saveas(fig4b, 'Fig4b_Errors.png');

% --- Fig 4c: Control Inputs ---
fig4c = figure('Color','w','Name','Fig 4c: Control Inputs','Position',[750,100,600,800]);
tau_labels = {'f_x (N)', 'f_y (N)', 'f_z (N)', '\tau_{\psi} (N\cdot m)'};
for i = 1:4
    subplot(4,1,i);
    plot(t, tau(:,i), 'b-', 'LineWidth', 1.0); % 控制力/力矩[cite: 2]
    ylabel(tau_labels{i}); grid on; xlim([0 200]);
end
xlabel('Time (sec)');
saveas(fig4c, 'Fig4c_ControlInputs.png');

disp('>>> 论文同款图像 (Fig 4a, 4b, 4c) 已生成并保存为 PNG。');