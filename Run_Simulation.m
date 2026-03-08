% Run_Simulation.m
clc; clear; close all;
run('Init_AUV_Params.m');
model_name = 'Adaptive_Fuzzy_HVS';
load_system(model_name);
set_param(model_name, 'UnderspecifiedInitializationDetection', 'Simplified');
% 使用刚性求解器 ode15s，大幅提升稳定性
set_param(model_name, 'Solver', 'ode15s');      
set_param(model_name, 'MaxStep', '0.01');   
set_param(model_name, 'StopTime', '120');     
save_system(model_name);
disp('✅ 配置完成！请直接运行 Plot_Results.m 启动仿真。');