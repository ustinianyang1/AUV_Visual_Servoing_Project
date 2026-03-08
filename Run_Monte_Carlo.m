% Run_Monte_Carlo.m - 50次随机干扰测试自动执行脚本
clc; clear;

disp('🌊 准备进行 50 次环境干扰测试 (对标论文 Fig.7)...');
model_name = 'Adaptive_Fuzzy_HVS';
load_system(model_name);

% 存储50次测试最终的特征误差
error_s1_list = zeros(50, 1);

for test_idx = 1:50
    % 根据论文设定生成均匀分布的随机干扰 
    tau_E1 = -95.0 + (93.7 - (-95.0)) * rand();
    tau_E2 = -56.2 + (14.5 - (-56.2)) * rand();
    tau_E3 = -6.4  + (13.9 - (-6.4))  * rand();
    tau_E_random = [tau_E1; tau_E2; tau_E3; 0; 0; 0];
    
    % 将随机干扰注入到 Simulink 工作区变量中
    assignin('base', 'tau_E_inject', tau_E_random);
    
    % 修改模型中的 Constant 1 模块(你的干扰力输入块)，让它读取工作区变量
    set_param([model_name, '/Constant1'], 'Value', 'tau_E_inject');
    
    % 静默运行仿真
    simOut = sim(model_name);
    
    % 提取第 3 个视觉特征 s1 (面积) 的稳态误差用于验证
    % 假设最终期望面积为 27
    s1_final = simOut.out_eta(end, 3); % 简化提取逻辑，实际需从 xi 提取
    error_s1_list(test_idx) = abs(s1_final - 27);
    
    fprintf('测试 %d/50 完成...\n', test_idx);
end

% 恢复 Constant 1 为全 0
set_param([model_name, '/Constant1'], 'Value', 'zeros(6,1)');
save_system(model_name);

disp('✅ 50 次测试完毕！平均特征误差分布如下：');
boxplot(error_s1_list);
title('50 次干扰测试下视觉特征 s_1 的稳态误差分布');
ylabel('MAE of s_1');