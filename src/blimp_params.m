% 飞艇参数.m
% 该脚本初始化室内飞艇的物理和流体动力学参数
% 基于“室内飞艇的自适应输出反馈轨迹跟踪控制”

%% 质量特性
mR = 0.884;     % 刚体质量（kg）
mAx = 0.585;    % 增加的质量x (kg)
mAy = 0.585;    % 添加质量 y (kg)
mAz = 0.607;    % z 轴上的附加质量 (kg)

IRBz = 0.039;   % 绕 z 轴的机构转动惯性量 (kg*m^2)
IAz = 0.012;    % z 周围的附加转动惯量 (kg*m^2)

xg = 0.05;      % x 轴重心偏移 (m)

% 总质量变量
mx = mR + mAx;
my = mR + mAy;
mz = mR + mAz;
Iz = IRBz + IAz;

%% 系统矩阵
% 惯性矩阵 M (4x4)
% 主要考虑 (x, y, z, r) 或 (u, v, w, p) 代表速度
% 由于 x_g = 0.05，v 轴和偏航 r 之间存在耦合。
M_mat = [mx, 0, 0, 0;
         0, my, 0, mR * xg;
         0, 0, mz, 0;
         0, mR * xg, 0, Iz];

% 分布矩阵 D (4x4)
Dvx = 0.031;
Dvy = 0.031;
Dvz = 0.025;
Dwz = 0.001;

D_mat = diag([Dvx, Dvy, Dvz, Dwz]);

%% 打包成一个结构以便于传递到常微分方程
params.mR = mR;
params.xg = xg;
params.mx = mx;
params.my = my;
params.mz = mz;
params.Iz = Iz;
params.M = M_mat;
params.D = D_mat;

disp('Blimp parameters loaded successfully.');
