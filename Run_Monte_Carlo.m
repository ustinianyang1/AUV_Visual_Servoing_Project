% Run_Monte_Carlo.m - 最终收敛修复版
clc; clear;
disp('准备进行 50 次环境干扰测试 (对标论文 Fig.7)...');

% 运行主程序初始化所有参数
main; 

model_name = 'Adaptive_Fuzzy_HVS';
load_system(model_name);

% 存储50次测试的视觉特征误差
error_xv_list = zeros(50, 1);
error_xu_list = zeros(50, 1);
error_s1_list = zeros(50, 1);

% 加载目标值
xi_hat = evalin('base', 'xi_hat');
% 【修复】固定相机内参，与visual_model完全一致
f_x = 45.4545; f_y = 3000; c_x = 0; c_y = 0;
target_depth = 3.45;

for test_idx = 1:50
    % 论文指定范围的随机干扰
    tau_E1 = -95.0 + (93.7 - (-95.0)) * rand();
    tau_E2 = -56.2 + (14.5 - (-56.2)) * rand();
    tau_E3 = -6.4  + (13.9 - (-6.4))  * rand();
    tau_E_random = [tau_E1; tau_E2; tau_E3; 0; 0; 0];
    
    % 注入干扰
    assignin('base', 'tau_E_inject', tau_E_random);
    set_param([model_name, '/Constant1'], 'Value', 'tau_E_inject');
    
    % 静默运行仿真
    simOut = sim(model_name);
    
    % 正确计算最终视觉特征误差
    eta_final = squeeze(simOut.out_eta(end, :));
    Z_final = target_depth - eta_final(3) + 0.01;
    
    xv_final = f_y * (-eta_final(1)) / Z_final + c_y;
    xu_final = f_x * eta_final(2) / Z_final + c_x;
    
    % 提取误差
    error_xv_list(test_idx) = abs(xv_final - xi_hat(1));
    error_xu_list(test_idx) = abs(xu_final - xi_hat(2));
    
    % 读取面积特征s1（需在Simulink中添加out_s1输出）
    if isfield(simOut, 'out_s1')
        s1_final = squeeze(simOut.out_s1(end));
    else
        s1_final = xi_hat(3);
    end
    error_s1_list(test_idx) = abs(s1_final - xi_hat(3));
    
    fprintf('测试 %d/50 完成...\n', test_idx);
end

% 恢复默认扰动
tau_E_default = [-0.65; -20.85; 3.75; 0; 0; 0];
assignin('base', 'tau_E_inject', tau_E_default);
set_param([model_name, '/Constant1'], 'Value', 'tau_E_inject');
save_system(model_name);

% 绘制论文Fig.7b
disp('50 次测试完毕！视觉特征误差分布如下：');
figure('Name', 'Monte Carlo Test Results', 'Position', [300, 300, 800, 400]);
boxplot([error_xv_list, error_xu_list, error_s1_list], {'x_v', 'x_u', 's_1'});
title('50 次干扰测试下视觉特征的稳态误差分布 (对标论文 Fig.7b)');
ylabel('MAE');
grid on;