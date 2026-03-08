% --- 1. 核心参数与初始状态计算 ---
num_rules = 17;
rng(0); q_j = -4 + 8 * rand(18, num_rules);

% 初始与目标状态 (论文Tab.III 100%还原)
eta_0 = [1.9; 4.5; 0.5; 0; 0; -1.87]; % 论文原始初始位姿
nu_0 = zeros(6,1);
xi_hat = [120; 160; 27; 0; 0; -1.87];  % 论文目标视觉特征
eta_d = [-0.04; 3.52; 3.45; 0; 0; -1.87]; % 论文目标位姿

% 【修复】计算自洽的初始面积 s1_0
% 目标深度：AUV到达目标位姿时，Z=0.01m，对应面积s1=27
target_Z = 0.01;
% 初始深度
initial_Z = 3.45 - eta_0(3) + 0.01;
% 面积与深度成反比，计算初始面积
s1_0 = 27 * (target_Z / initial_Z); 

% 其他初始值
phi_0 = zeros(num_rules, 6);
e2_hat_0 = zeros(6,1);

% 默认环境扰动 (论文中等强度扰动)
tau_E_inject = [-0.65; -20.85; 3.75; 0; 0; 0];

% --- 2. 加载模型与强制仿真配置 ---
model_name = 'Adaptive_Fuzzy_HVS';
load_system(model_name);

% 强制论文要求的仿真设置
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.01');
set_param(model_name, 'StopTime', '120');

% 配置积分器初始值
set_param([model_name, '/Int_eta'], 'InitialCondition', 'eta_0');
set_param([model_name, '/Int_nu'], 'InitialCondition', 'nu_0');
set_param([model_name, '/Int_s1'], 'InitialCondition', 's1_0');
set_param([model_name, '/Int_phi'], 'InitialCondition', 'phi_0');
set_param([model_name, '/Int_e2hat'], 'InitialCondition', 'e2_hat_0');
set_param([model_name, '/Memory'], 'InitialCondition', 'zeros(6,1)');

% 配置常量模块
set_param([model_name, '/xi_hat'], 'Value', 'xi_hat');
set_param([model_name, '/q_j'], 'Value', 'q_j');
% 惯性矩阵逆（与auv_plant硬编码参数完全匹配）
M_mat = diag([98+49, 98+49, 98+49, 8, 8, 8]);
M_inv = inv(M_mat);
assignin('base', 'M_inv', M_inv);
assignin('base', 'tau_E_inject', tau_E_inject);
assignin('base', 'eta_d', eta_d); % 给绘图函数用
set_param([model_name, '/M_inv'], 'Value', 'M_inv');
set_param([model_name, '/Constant1'], 'Value', 'tau_E_inject');

% --- 3. 运行仿真 ---
disp('采用 0.01s 定步长 ode4 仿真，正在运行...');
simOut = sim(model_name);
disp('仿真完成，准备出图！');

% --- 4. 绘图 ---
Plot_Results(simOut);