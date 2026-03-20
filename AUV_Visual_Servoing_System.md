这是一份为你重新整理好的 `AUV_Visual_Servoing_System.md` 完整文档。

我把模型当前的真实拓扑架构（多算法切换模式）以及你提供的这 9 个最新、最稳定的核心代码全部整合进去了。代码没有任何省略，你可以直接一键复制并保存为 `.md` 文件，作为你这套仿真系统的最终版技术说明书。

```markdown
# AUV 凸优化自适应模糊视觉伺服控制系统仿真模型文档

*(基于 IEEE TCE 论文的最终多算法综合对比验证版)*

## 一、核心连线逻辑与架构 (多算法切换综合仿真平台)

本架构彻底规避了 Simulink 的“代数环”死锁问题，根除了 MATLAB Function 中的持久状态变量报错，并集成了 4 种控制算法以供横向对比验证。系统由 4 个连续时间积分器（`1/s`）、1 个算法切换枢纽、以及多套平行的控制算法模块组成。

### 1. 积分器 (Integrators) 及其初始条件配置
*(仅服务于被控对象和提出的自适应控制律)*
* **Integrator_Eta (位姿积分器)**：初始条件 `[1.9; 4.5; 0.5; 0; 0; -1.87]` （这是实验起始点，切勿填成目标点）。
* **Integrator_V (速度积分器)**：初始条件 `zeros(6,1)`。
* **Integrator_Phi (模糊权重积分器)**：初始条件 `zeros(17,6)`。
* **Integrator_E2Hat (指令滤波积分器)**：初始条件 `zeros(6,1)`。

### 2. 全局信号源与切换枢纽
* **Target_Pose (常数模块 xi_d)**
  * 输出：`[120; 160; 27; 0; 0; -1.87]` (作为目标视觉特征，并行送入三种对比算法)。
* **Algorithm_Selector (常数模块 - 算法选择器)**
  * 作用：输出 `1`, `2`, `3`, 或 `4`，送入 `Controller_Switch` 的顶端控制端口。
* **Controller_Switch (Multiport Switch)**
  * **控制端 (Port 1)**：接 `Algorithm_Selector`。
  * **数据端 (Port 2-5)**：分别接 4 种控制算法输出的控制力矩 `tau`。
  * **输出**：最终选定的控制力矩 `tau`，连入 `AUV_Dynamics`。

### 3. 核心模块信号拓扑 (按数据流向)

**【被控对象与视觉映射】**
* **1. AUV_Dynamics**
  * 输入：`v`, `eta`, `tau` (来自 Controller_Switch 的统一输出)
  * 输出：`dot_eta` (连入 Integrator_Eta), `dot_v` (连入 Integrator_V)
* **2. Visual_Model (刚性几何相机)**
  * 输入：`eta` (来自 Integrator_Eta)
  * 输出：`xi` (实际视觉特征，广播给所有控制器)

**【控制器 1：Proposed Control (提议算法，包含4个子模块)】**
* **3. Virtual_Control**
  * 输入：`xi`, `v`, `eta`
  * 输出：`e1`, `alpha1`, `J_xi` (悬空)
* **4. Command_Filter**
  * 输入：`v`, `alpha1`, `e2_hat` (来自 Integrator_E2Hat)
  * 输出：`dot_e2_hat` (连入 Integrator_E2Hat), `e2`
* **5. Fuzzy_Update (彻底斩断代数环版)**
  * 输入：`eta`, `v`, `e2`, `dot_e2_hat`
  * 输出：`dot_phi` (连入 Integrator_Phi)
* **6. Actual_Control (彻底斩断代数环版)**
  * 输入：`phi` (来自 Integrator_Phi), `e2`, `eta`, `v`
  * 输出：`tau` (连入 Controller_Switch 的 Port 2)

**【控制器 2-4：Baseline 对比算法】**
*(这三个模块共享输入源：`xi`, `v`, `eta`, `xi_d`)*
* **7. ACFBC_Controller (自适应指令滤波)**
  * 输出：`tau` (连入 Controller_Switch 的 Port 3)
* **8. PID_Controller (经典 PID)**
  * 输出：`tau` (连入 Controller_Switch 的 Port 4)
* **9. LCANNC_Controller (低复杂度神经网络)**
  * 输出：`tau` (连入 Controller_Switch 的 Port 5)

---

## 二、仿真配置与参数设置清单

必须使用强制的硬实时定步长环境，严禁使用变步长：

* **Solver Type (求解器类型)**：`Fixed-step` (定步长)
* **Solver (具体算法)**：`ode4 (Runge-Kutta)`
* **Fixed-step size (步长)**：`0.01` 秒
* **Stop time (终止时间)**：`120` 秒

---

## 三、核心模块 MATLAB Function 完整源代码

### 1. AUV_Dynamics.m (本体非线性动力学)

```matlab
function [dot_eta, dot_v] = AUV_Dynamics(v, eta, tau)
    % -------------------------------------------------------------
    % 表 II 物理模型参数硬编码 (Girona 500)
    % -------------------------------------------------------------
    m = 98.0;              % AUV 质量 (kg)
    W = 961.38;            % 重力 (N)
    B = 952.56;            % 浮力 (N)
    z_G = 0.05;            % 重心Z轴偏移量 (m)
    
    % 刚体转动惯量与线加速度附加质量
    Ix = 8.0; Iy = 8.0; Iz = 8.0;
    Xu_dot = 49.0; Yv_dot = 49.0; Zw_dot = 49.0;
    
    % 组合构建全局对角惯性矩阵 M (6x6)
    M11 = m + Xu_dot;     M22 = m + Yv_dot;     M33 = m + Zw_dot;
    M = diag([M11, M22, M33, Ix, Iy, Iz]);
    
    % 线性阻尼与二次非线性阻尼参数
    Xuu = 148.0; Yvv = 148.0; Zww = 148.0;
    Kp = 130.0; Mq = -130.0; Nr = -130.0;
    Kpp = 180.0; Mqq = -180.0; Nrr = -180.0;
    
    u = v(1); vv = v(2); w = v(3); 
    p = v(4); q = v(5); r = v(6);
    
    % 构建流体阻尼矩阵 D(v)
    D = diag([148.0*abs(u), 148.0*abs(vv), 148.0*abs(w), ...
              130.0+180.0*abs(p), 130.0+180.0*abs(q), 130.0+180.0*abs(r)]);
              
    % 构建科里奥利力与向心力矩阵 C(v)
    C = zeros(6,6);
    C(1,5) = M33*w;       C(1,6) = -M22*vv;
    C(2,4) = -M33*w;      C(2,6) = M11*u;
    C(3,4) = M22*vv;      C(3,5) = -M11*u;
    C(4,2) = M33*w;       C(4,3) = -M22*vv; C(4,5) = Iz*r;    C(4,6) = -Iy*q;
    C(5,1) = -M33*w;      C(5,3) = M11*u;   C(5,4) = -Iz*r;   C(5,6) = Ix*p;
    C(6,1) = M22*vv;      C(6,2) = -M11*u;  C(6,4) = Iy*q;    C(6,5) = -Ix*p;
    
    % 计算重力与浮力产生的恢复力矩向量 g(eta)
    phi = eta(4); theta = eta(5); psi = eta(6);
    g_eta = zeros(6,1);
    g_eta(1) = (W - B) * sin(theta);
    g_eta(2) = -(W - B) * cos(theta) * sin(phi);
    g_eta(3) = -(W - B) * cos(theta) * cos(phi);
    g_eta(4) = (z_G * W) * cos(theta) * sin(phi);
    g_eta(5) = (z_G * W) * sin(theta);
    g_eta(6) = 0;
    
    % 添加外部动态未知扰动 tau_E
    tau_E = [90*sin(0.1*1); 10*cos(0.1*1); 5*sin(0.1*1); 0; 0; 0];
    
    % 计算欧拉角位姿的运动学雅可比矩阵 J_eta
    J_eta = zeros(6,6);
    J_eta(1:3, 1:3) = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
                       sin(psi)*cos(theta),  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
                       -sin(theta),           cos(theta)*sin(phi),                             cos(theta)*cos(phi)];
    J_eta(4:6, 4:6) = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                       0, cos(phi),           -sin(phi);
                       0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
                       
    % 导出加速度状态方程
    dot_v = M \ (tau + tau_E - C*v - D*v - g_eta);
    % 导出位姿状态方程
    dot_eta = J_eta * v;
end
```

### 2. Visual_Model.m (刚性几何相机-极性修正版)

```matlab
function xi = Visual_Model(eta)
    % -------------------------------------------------------------
    % 刚性几何相机传感器模型：已严格修正投影极性，恢复负反馈稳定闭环
    % -------------------------------------------------------------
    % 稳态目标物理位置
    x_target = -0.04; 
    y_target = 3.52; 
    
    x = eta(1); y = eta(2); z = eta(3);
    phi = eta(4); theta = eta(5); psi = eta(6);
    
    % 深度计算
    Z = 5.0 - z;
    if Z < 0.1
        Z = 0.1;
    end
    
    % 物理偏差
    err_x = x - x_target;
    err_y = y - y_target;
    
    % 转换到 AUV 本体坐标系 (Body Frame)
    body_err_x = cos(psi) * err_x + sin(psi) * err_y;
    body_err_y = -sin(psi) * err_x + cos(psi) * err_y;
    
    % 提取论文给定的相机焦距参数
    fx = 200.0; fy = 200.0;
    
    % =========================================================
    % 【致命 Bug 修复区】：必须是减号！
    % 原理：当 AUV 往前走（机体 X 误差靠近 0）时，特征像素必须反向收敛
    % =========================================================
    xv = 120 - fy * (body_err_x / Z);
    xu = 160 - fx * (body_err_y / Z);
    
    % 面积投影规律保持不变
    s1 = 27.0 * (1.55 / Z)^2;
    
    xi = [xv; xu; s1; phi; theta; psi];
end
```

### 3. Virtual_Control.m (反步法第一级伪逆指令生成)

```matlab
function [e1, alpha1, J_xi] = Virtual_Control(xi, v, eta)
    % -------------------------------------------------------------
    % 表 III 控制律参数与目标位姿硬编码
    % -------------------------------------------------------------
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
    
    % 第一级误差计算
    e1 = xi - xi_hat;
    
    % 虚拟控制律生成：引入 Moore-Penrose 广义伪逆
    alpha1 = -pinv(J_xi) * (k1 * e1 - dot_xi_hat);
end
```

### 4. Command_Filter.m (反步法指令微分滤波器)

```matlab
function [dot_e2_hat, e2] = Command_Filter(v, alpha1, e2_hat)
    % -------------------------------------------------------------
    % 表 III 滤波及速度跟踪参数
    % -------------------------------------------------------------
    b1 = 50.0;
    
    % 提取反步法第二步的核心对象
    e2 = v - alpha1;
    
    % 估算并输出经过平滑的跟踪误差导数
    dot_e2_hat = -b1 * (e2_hat - e2);
end
```

### 5. Fuzzy_Update.m (凸优化权重自适应更新律)

```matlab
function dot_phi = Fuzzy_Update(eta, v, e2, dot_e2_hat)
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
        % 【关键修复区】：强行加上梯度下降的负号，将正反馈修正为负反馈
        multiplier = -k3(i) * (dot_e2_hat(i) + k2 * M_inv(i) * e2(i));
        dot_phi(:, i) = multiplier * S_gamma;
    end
end
```

### 6. Actual_Control.m (底层扭矩控制合成输出)

```matlab
function tau = Actual_Control(phi, e2, eta, v)
    % -------------------------------------------------------------
    % 内部重新计算 dot_eta，彻底斩断 Simulink 外部代数环
    % -------------------------------------------------------------
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
    
    % -------------------------------------------------------------
    % 消除 rng/rand 带来的持久状态报错，使用确定性伪随机矩阵硬生成
    % -------------------------------------------------------------
    q_j = zeros(18, n_rules);
    for i = 1:18
        for j = 1:n_rules
            val = sin(i * 12.345 + j * 67.89);
            frac = abs(val) - floor(abs(val));
            q_j(i, j) = -4 + 8 * frac;
        end
    end
    
    % 计算高斯基函数 S_gamma
    S_gamma = zeros(n_rules, 1);
    for j = 1:n_rules
        diff = gamma - q_j(:, j);
        S_gamma(j) = exp(-(diff'*diff) / (2.0^2));
    end
    
    sum_S = sum(S_gamma);
    if sum_S > 0
        S_gamma = S_gamma / sum_S;
    end
    
    % 输出综合扭矩 tau
    tau_fuzzy = phi' * S_gamma;
    tau = tau_fuzzy - k2 * e2;
end
```

### 7. PID_Controller.m (经典 PID 算法对比版)

```matlab
function tau = fcn(xi, v, eta, xi_d)
    persistent integral_e prev_e
    if isempty(integral_e)
        integral_e = zeros(6,1);
        prev_e = zeros(6,1);
    end
    dt = 0.01;
    e = xi - xi_d;
    
    integral_e = integral_e + e * dt;
    integral_e = max(min(integral_e, 300), -300); % 积分抗饱和
    de = (e - prev_e) / dt;
    prev_e = e;
    
    % 重构：放大平移增益，补齐原论文缺失的角度控制律，呈现正常的滞后/超调现象
    Scale = 500;
    Kp = diag([0.003*Scale, 0.003*Scale, 0.0001*Scale, 10, 10, 10]);
    Ki = diag([0.0001*Scale, 0.0001*Scale, 0.00001*Scale, 0.5, 0.5, 0.5]);
    Kd = diag([0.0045*Scale, 0.0045*Scale, 0.003*Scale, 10, 10, 10]);
    
    tau = Kp * e + Ki * integral_e + Kd * de;
    tau = max(min(tau, 800), -800); % 物理限幅
end
```

### 8. ACFBC_Controller.m (自适应指令滤波算法对比版)

```matlab
function tau = fcn(xi, v, eta, xi_d)
    persistent z2_hat
    if isempty(z2_hat)
        z2_hat = zeros(6,1);
    end
    dt = 0.01;
    k1 = 2; k2 = 50; g_min = 1.0; epsilon = 1.0; b1 = 20; % 调大 epsilon 消除打带抖振
    
    x_v = xi(1); x_u = xi(2); s1 = xi(3); phi = xi(4); theta = xi(5); psi = xi(6);
    cy = 120; cx = 160; fy = -200; fx = -200;
    a1 = x_v - cy; a2 = x_u - cx;
    Z = 1.55 / sqrt(abs(s1) / 27.0 + 1e-4); 
    if Z < 0.1; Z = 0.1; end
    L = [-fy/Z, 0, a1/Z, a1*a2/fx, -(fy^2+a1^2)/fy, a2*fy/fx;
         0, -fx/Z, a2/Z, (fx^2+a2^2)/fx, -a1*a2/fy, -a1*fx/fy];
    J3 = [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi), -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    J = [L; (2*s1/Z)*J3, zeros(1,3); zeros(3,3), J2];
    
    z1 = xi - xi_d;
    J_pinv = pinv(J);
    alpha1 = -J_pinv * (k1 * z1);
    alpha1 = max(min(alpha1, 1.0), -1.0); % 限制虚拟指令，防止系统崩溃
    
    z2 = v - alpha1;
    dot_z2_hat = -b1 * (z2_hat - z2);
    z2_hat = z2_hat + dot_z2_hat * dt;
    
    alpha_bar_2 = J' * z1 + k2 * z2 + dot_z2_hat;
    norm_z2 = norm(z2); norm_ab2 = norm(alpha_bar_2);
    
    % 重构：补齐被原论文省略的标称反馈项 -k2*z2，否则系统无基础阻尼
    robust_term = -z2 * (norm_ab2^2) / (g_min * sqrt((norm_z2^2) * (norm_ab2^2) + epsilon^2));
    tau = -k2 * z2 + robust_term;
    tau = max(min(tau, 800), -800);
end
```

### 9. LCANNC_Controller.m (低复杂度神经网络算法对比版)

```matlab
function tau = fcn(xi, v, eta, xi_d)
    persistent Q_m W_varpi
    if isempty(Q_m)
        Q_m = 0; W_varpi = 0;
    end
    dt = 0.01;
    k1 = 2; k2 = 50; l1 = 1; l2 = 1; r1 = 0.1; m1 = 0.01; r2 = 5; m2 = 5;
    
    x_v = xi(1); x_u = xi(2); s1 = xi(3); phi = xi(4); theta = xi(5); psi = xi(6);
    cy = 120; cx = 160; fy = -200; fx = -200;
    a1 = x_v - cy; a2 = x_u - cx;
    Z = 1.55 / sqrt(abs(s1) / 27.0 + 1e-4);
    if Z < 0.1; Z = 0.1; end
    L = [-fy/Z, 0, a1/Z, a1*a2/fx, -(fy^2+a1^2)/fy, a2*fy/fx;
         0, -fx/Z, a2/Z, (fx^2+a2^2)/fx, -a1*a2/fy, -a1*fx/fy];
    J3 = [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi), -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    J = [L; (2*s1/Z)*J3, zeros(1,3); zeros(3,3), J2];
    
    z1 = xi - xi_d;
    J_pinv = pinv(J);
    alpha1 = -J_pinv * (k1 * z1);
    alpha1 = max(min(alpha1, 1.0), -1.0);
    
    z2 = v - alpha1;
    sigma_bar = [eta; v];
    
    % 重构：引入宽度缩放因子 (30.0)，否则范数过大会导致 exp(-x) 极速下溢为 0，网络无法更新
    Theta_T_Theta = exp(-(sigma_bar'*sigma_bar) / 30.0); 
    
    sum_z2_sq = sum(z2.^2);
    dot_Q_m = (r2 * sum_z2_sq) / (2 * l2^2) - m2 * Q_m;
    Q_m = Q_m + dot_Q_m * dt;
    
    dot_W_varpi = (r1 * sum_z2_sq * Theta_T_Theta) / (2 * l1^2) - m1 * W_varpi;
    W_varpi = W_varpi + dot_W_varpi * dt;
    
    tau = -k2 * z2 - z2 * W_varpi * Theta_T_Theta / (2 * l1^2) - z2 * Q_m / (2 * l2^2);
    tau = max(min(tau, 800), -800);
end
```
```