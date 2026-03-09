% =========================================================================
% Convex Optimization-Based Adaptive Fuzzy Visual Servoing for AUVs
% 完全还原脚本：主函数 main.m
% 强制性规范：ode4定步长0.01秒，停止时间120秒
% =========================================================================
clear; clc; close all;

% 在工作区预先加载Simulink系统所需的全局标量(部分初始化支持)
disp('正在初始化 AUV 视觉伺服控制系统参数...');

% 指定将要创建或打开的 Simulink 模型名称
modelName = 'AUV_Visual_Servoing_System';

% 如果模型未打开，则初始化一个新的空白系统用于挂载
if ~bdIsLoaded(modelName)
    try
        open_system(modelName);
    catch
        disp('未找到现成模型，正在创建新模型...');
        new_system(modelName);
        open_system(modelName);
    end
end

% 【核心指令】强制覆盖Simulink的求解器及仿真参数配置
set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'Solver', 'ode4');      % 使用四阶Runge-Kutta算法
set_param(modelName, 'FixedStep', '0.01');   % 步长硬编码为10ms
set_param(modelName, 'StopTime', '120');     % 停止时间定为120秒

disp('仿真配置完成：Fixed-step 0.01s, ODE4, StopTime 120s. 准备搭建模块...');