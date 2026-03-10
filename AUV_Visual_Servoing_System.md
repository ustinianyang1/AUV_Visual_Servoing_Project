# AUV 凸优化自适应模糊视觉伺服控制系统仿真模型文档

*(基于 IEEE TCE 论文的最终稳定复刻版 - 零稳态误差修正版)*

## 一、核心连线逻辑与架构 (6模块 + 4积分器)

本架构彻底规避了 Simulink 的“代数环”死锁问题，根除了 MATLAB Function 中的持久状态变量报错，并去除了会导致假性稳态误差的视觉特征积分器。整个闭环系统由 4 个连续时间积分器（`1/s`）和 6 个核心函数块组成。

### 1. 积分器 (Integrators) 及其初始条件配置

* **Integrator_Eta (位姿积分器)**：初始条件 `[1.9; 4.5; 0.5; 0; 0; -1.87]` （这是实验起始点，切勿填成目标点）。
* **Integrator_V (速度积分器)**：初始条件 `zeros(6,1)`。
* **Integrator_Phi (模糊权重积分器)**：初始条件 `zeros(17,6)`。
* **Integrator_E2Hat (指令滤波积分器)**：初始条件 `zeros(6,1)`。

### 2. 核心模块信号拓扑

* **1. AUV_Dynamics**
* 输入：`v` (来自 Integrator_V), `eta` (来自 Integrator_Eta), `tau` (来自模块 6)
* 输出：`dot_eta` (连入 Integrator_Eta), `dot_v` (连入 Integrator_V)


* **2. Visual_Model (刚性几何相机)**
* 输入：`eta` (来自 Integrator_Eta)
* 输出：`xi` (连入模块2)


* **3. Virtual_Control**
* 输入：`xi` (来自模块 2), `v` (来自 Integrator_V), `eta` (来自 Integrator_Eta)
* 输出：`e1` (可接 To Workspace), `alpha1` (连入模块 4), `J_xi` (悬空)


* **4. Command_Filter**
* 输入：`v` (来自 Integrator_V), `alpha1` (来自模块 3), `e2_hat` (来自 Integrator_E2Hat)
* 输出：`dot_e2_hat` (连入 Integrator_E2Hat), `e2` (连入模块 5 和 6)


* **5. Fuzzy_Update (彻底斩断代数环版)**
* 输入：`eta` (来自 Integrator_Eta), `v` (来自 Integrator_V), `e2` (来自模块 4), `dot_e2_hat` (来自模块 4)
* 输出：`dot_phi` (连入 Integrator_Phi)


* **6. Actual_Control (彻底斩断代数环版)**
* 输入：`phi` (来自 Integrator_Phi), `e2` (来自模块 4), `eta` (来自 Integrator_Eta), `v` (来自 Integrator_V)
* 输出：`tau` (连回模块 1，并可接 To Workspace)



---

## 二、仿真配置与参数设置清单

### 求解器配置 (Solver Settings)

必须使用强制的硬实时定步长环境，严禁使用变步长：

* **Solver Type (求解器类型)**：`Fixed-step` (定步长)
* **Solver (具体算法)**：`ode4 (Runge-Kutta)`
* **Fixed-step size (步长)**：`0.01` 秒
* **Stop time (终止时间)**：`120` 秒

*(注：也可通过运行配套的 `main.m` 脚本，将上述配置一键强制写入 Simulink 模型内核。)*

---

## 三、核心模块 MATLAB Function 完整源代码

### 1. AUV_Dynamics.m (本体非线性动力学)

```matlab
function [dot_eta, dot_v] = AUV_Dynamics(v, eta, tau)
    m = 98.0; W = 961.38; B = 952.56; z_G = 0.05;
    Ix = 8.0; Iy = 8.0; Iz = 8.0;
    Xu_dot = 49.0; Yv_dot = 49.0; Zw_dot = 49.0;
    
    M11 = m + Xu_dot; M22 = m + Yv_dot; M33 = m + Zw_dot;
    M = diag([M11, M22, M33, Ix, Iy, Iz]);
    
    u = v(1); vv = v(2); w = v(3); 
    p = v(4); q = v(5); r = v(6);
    
    D = diag([148.0*abs(u), 148.0*abs(vv), 148.0*abs(w), ...
              130.0+180.0*abs(p), 130.0+180.0*abs(q), 130.0+180.0*abs(r)]);
              
    C = zeros(6,6);
    C(1,5) = M33*w;       C(1,6) = -M22*vv;
    C(2,4) = -M33*w;      C(2,6) = M11*u;
    C(3,4) = M22*vv;      C(3,5) = -M11*u;
    C(4,2) = M33*w;       C(4,3) = -M22*vv; C(4,5) = Iz*r;    C(4,6) = -Iy*q;
    C(5,1) = -M33*w;      C(5,3) = M11*u;   C(5,4) = -Iz*r;   C(5,6) = Ix*p;
    C(6,1) = M22*vv;      C(6,2) = -M11*u;  C(6,4) = Iy*q;    C(6,5) = -Ix*p;
    
    phi = eta(4); theta = eta(5); psi = eta(6);
    g_eta = zeros(6,1);
    g_eta(1) = (W - B) * sin(theta);
    g_eta(2) = -(W - B) * cos(theta) * sin(phi);
    g_eta(3) = -(W - B) * cos(theta) * cos(phi);
    g_eta(4) = (z_G * W) * cos(theta) * sin(phi);
    g_eta(5) = (z_G * W) * sin(theta);
    g_eta(6) = 0;
    
    % 洋流扰动置零，用于获取完美的稳态收敛精度测试
    tau_E = zeros(6,1);
    
    J_eta = zeros(6,6);
    J_eta(1:3, 1:3) = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
                       sin(psi)*cos(theta),  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
                       -sin(theta),           cos(theta)*sin(phi),                             cos(theta)*cos(phi)];
    J_eta(4:6, 4:6) = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                       0, cos(phi),           -sin(phi);
                       0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
                       
    dot_v = M \ (tau + tau_E - C*v - D*v - g_eta);
    dot_eta = J_eta * v;
end

```

### 2. Visual_Model.m (刚性几何相机-修正极性闭环版)

```matlab
function xi = Visual_Model(eta)
    % -------------------------------------------------------------
    % 刚性几何相机传感器模型：直接映射，彻底消除微积分截断漂移误差
    % -------------------------------------------------------------
    % 稳态目标物理位置硬编码
    x_target = -0.04; 
    y_target = 3.52; 
    
    x = eta(1); y = eta(2); z = eta(3);
    phi = eta(4); theta = eta(5); psi = eta(6);
    
    % 海底深度近似 (水深5.0m)
    Z = 5.0 - z;
    if Z < 0.1
        Z = 0.1;
    end
    
    % 计算偏差并旋转至载体坐标系
    err_x = x - x_target;
    err_y = y - y_target;
    body_err_x = cos(psi) * err_x + sin(psi) * err_y;
    body_err_y = -sin(psi) * err_x + cos(psi) * err_y;
    
    fx = 200.0; fy = 200.0;
    
    % 像素坐标映射 (使用负号确保构筑负反馈收敛系统)
    xv = 120 - fy * (body_err_x / Z);
    xu = 160 - fx * (body_err_y / Z);
    
    % 缩放目标面积衰减规律
    s1 = 27.0 * (1.55 / Z)^2;
    
    % 输出静态无漂移的特征状态
    xi = [xv; xu; s1; phi; theta; psi];
end

```

### 3. Virtual_Control.m (反步法第一级伪逆指令生成)

```matlab
function [e1, alpha1, J_xi] = Virtual_Control(xi, v, eta)
    xi_hat = [120; 160; 27; 0; 0; -1.87];  
    dot_xi_hat = zeros(6,1);
    k1 = 2.0;                              
    
    fx = 200.0; fy = 200.0; cx = 160.0; cy = 120.0; 
    xv = xi(1); xu = xi(2); s1 = xi(3);
    phi = eta(4); theta = eta(5);
    
    Z = 5.0 - eta(3); if Z < 0.1; Z = 0.1; end
    a1 = xv - cy; a2 = xu - cx;
    
    L = [-fy/Z, 0, a1/Z, (a1*a2)/fx, -(fy^2 + a1^2)/fy, a2*(fy/fx);
          0, -fx/Z, a2/Z, (fx^2 + a2^2)/fx, -(a1*a2)/fy, -a1*(fx/fy)];
          
    J3_eta2 = [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    area_vec = (2 * s1 / Z) * J3_eta2;
    J2_eta2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi), -sin(phi);
               0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
               
    J_xi = zeros(6,6);
    J_xi(1:2, :) = L; 
    J_xi(3, 1:3) = area_vec;
    J_xi(4:6, 4:6) = J2_eta2;
    
    e1 = xi - xi_hat;
    % 利用伪逆抵御奇异位姿
    alpha1 = -pinv(J_xi) * (k1 * e1 - dot_xi_hat);
end

```

### 4. Command_Filter.m (反步法指令微分滤波器)

```matlab
function [dot_e2_hat, e2] = Command_Filter(v, alpha1, e2_hat)
    b1 = 50.0;
    e2 = v - alpha1;
    dot_e2_hat = -b1 * (e2_hat - e2);
end

```

### 5. Fuzzy_Update.m (凸优化权重自适应更新律)

```matlab
function dot_phi = Fuzzy_Update(eta, v, e2, dot_e2_hat)
    % 内部重新计算导数，物理隔离外部信号以破除 Simulink 代数环
    phi_ang = eta(4); theta = eta(5); psi = eta(6);
    J_eta = zeros(6,6);
    J_eta(1:3, 1:3) = [cos(psi)*cos(theta), -sin(psi)*cos(phi_ang)+cos(psi)*sin(theta)*sin(phi_ang),  sin(psi)*sin(phi_ang)+cos(psi)*cos(phi_ang)*sin(theta);
                       sin(psi)*cos(theta),  cos(psi)*cos(phi_ang)+sin(phi_ang)*sin(theta)*sin(psi), -cos(psi)*sin(phi_ang)+sin(theta)*sin(psi)*cos(phi_ang);
                       -sin(theta),           cos(theta)*sin(phi_ang),                             cos(theta)*cos(phi_ang)];
    J_eta(4:6, 4:6) = [1, sin(phi_ang)*tan(theta), cos(phi_ang)*tan(theta);
                       0, cos(phi_ang),           -sin(phi_ang);
                       0, sin(phi_ang)/cos(theta), cos(phi_ang)/cos(theta)];
    dot_eta = J_eta * v;
    gamma = [eta; dot_eta; v];

    k2 = 100.0;
    k3 = [90; 90; 5; 1; 1; 1];
    d_j = 2.0;                        
    n_rules = 17;
    M_inv = 1./ [147.0; 147.0; 147.0; 8.0; 8.0; 8.0];
    
    % 确定性伪随机矩阵硬生成（破除 Simulink 对 rng() 内部状态的编译报错）
    q_j = zeros(18, n_rules);
    for i = 1:18
        for j = 1:n_rules
            val = sin(i * 12.345 + j * 67.89);
            frac = abs(val) - floor(abs(val));
            q_j(i, j) = -4 + 8 * frac;
        end
    end
    
    S_gamma = zeros(n_rules, 1);
    for j = 1:n_rules
        diff = gamma - q_j(:, j);
        norm_sq = diff' * diff;
        S_gamma(j) = exp(-norm_sq / (d_j^2));
    end
    
    sum_S = sum(S_gamma);
    if sum_S > 0
        S_gamma = S_gamma / sum_S;
    end
    
    dot_phi = zeros(n_rules, 6);
    for i = 1:6
        % 【核心修复】：补偿梯度下降的负反馈负号
        multiplier = -k3(i) * (dot_e2_hat(i) + k2 * M_inv(i) * e2(i));
        dot_phi(:, i) = multiplier * S_gamma;
    end
end

```

### 6. Actual_Control.m (底层扭矩控制合成输出)

```matlab
function tau = Actual_Control(phi, e2, eta, v)
    % 内部重新计算导数，物理隔离外部信号以破除 Simulink 代数环
    phi_ang = eta(4); theta = eta(5); psi = eta(6);
    J_eta = zeros(6,6);
    J_eta(1:3, 1:3) = [cos(psi)*cos(theta), -sin(psi)*cos(phi_ang)+cos(psi)*sin(theta)*sin(phi_ang),  sin(psi)*sin(phi_ang)+cos(psi)*cos(phi_ang)*sin(theta);
                       sin(psi)*cos(theta),  cos(psi)*cos(phi_ang)+sin(phi_ang)*sin(theta)*sin(psi), -cos(psi)*sin(phi_ang)+sin(theta)*sin(psi)*cos(phi_ang);
                       -sin(theta),           cos(theta)*sin(phi_ang),                             cos(theta)*cos(phi_ang)];
    J_eta(4:6, 4:6) = [1, sin(phi_ang)*tan(theta), cos(phi_ang)*tan(theta);
                       0, cos(phi_ang),           -sin(phi_ang);
                       0, sin(phi_ang)/cos(theta), cos(phi_ang)/cos(theta)];
    dot_eta = J_eta * v;

    k2 = 100.0;
    gamma = [eta; dot_eta; v];
    n_rules = 17;
    
    % 确定性伪随机基底一致性获取
    q_j = zeros(18, n_rules);
    for i = 1:18
        for j = 1:n_rules
            val = sin(i * 12.345 + j * 67.89);
            frac = abs(val) - floor(abs(val));
            q_j(i, j) = -4 + 8 * frac;
        end
    end
    
    S_gamma = zeros(n_rules, 1);
    for j = 1:n_rules
        diff = gamma - q_j(:, j);
        S_gamma(j) = exp(-(diff'*diff) / (2.0^2));
    end
    
    sum_S = sum(S_gamma);
    if sum_S > 0
        S_gamma = S_gamma / sum_S;
    end
    
    % 模糊未知推力补偿 + 闭环速度误差矫正
    tau_fuzzy = phi' * S_gamma;
    tau = tau_fuzzy - k2 * e2;
end

```