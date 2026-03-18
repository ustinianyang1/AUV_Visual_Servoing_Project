% AUV visual servoing main script
clear; clc; close all;

% 初始化模型
disp('正在初始化 AUV 视觉伺服控制系统参数...');

modelName = 'AUV_Visual_Servoing_System';

if ~bdIsLoaded(modelName)
    try
        open_system(modelName);
    catch
        disp('未找到现成模型，正在创建新模型...');
        new_system(modelName);
        open_system(modelName);
    end
end

% 设置固定步长仿真参数
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'Solver', 'ode4');
set_param(modelName, 'FixedStep', '0.01');
set_param(modelName, 'StopTime', '120');

disp('仿真配置完成：Fixed-step 0.01s, ODE4, StopTime 120s. 准备搭建模块...');