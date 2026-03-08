# AUV 凸优化自适应模糊视觉伺服控制系统仿真模型文档
*(基于 IEEE TCE 论文的最终稳定复刻版)*

## 一、核心连线逻辑与架构
本架构彻底规避了 Simulink 的“代数环”死锁问题，并通过分离常数输入解决了 Coder 编译维度欠定的报错。整个系统高度模块化，主要由四个核心函数块组成。

### 模块连接关系
**1. auv_plant (AUV 本体动力学)**
* 输入 `tau`：来自 Memory 模块的输出 (核心：打断代数环)。
* 输入 `tau_E`：来自常量模块 (外部扰动 `tau_E_inject`)。
* 输入 `eta`、`nu`：分别来自 Int_eta、Int_nu 积分器。
* 输出 `eta_dot`：连接至 Int_eta 积分器、controller_fuzzy 模块、adaptive_update 模块。
* 输出 `nu_dot`：连接至 Int_nu 积分器。

**2. visual_model (极简视觉混合模型)**
* 输入 `eta`、`nu`：分别来自 Int_eta、Int_nu 积分器。
* 输入 `s1_current`：来自 Int_s1 积分器。
* 输出 `xi`、`J_xi`：连接至 controller_fuzzy 模块。
* 输出 `s1_dot`：连接至 Int_s1 积分器。

**3. controller_fuzzy (模糊控制器)**
* 输入 `xi`、`J_xi`：来自 visual_model 模块。
* 输入 `xi_hat`：来自常量模块 (目标视觉特征)。
* 输入 `nu`、`eta`：分别来自 Int_nu、Int_eta 积分器。
* 输入 `eta_dot`：来自 auv_plant 模块。
* 输入 `phi_weights`：来自 Int_phi 积分器。
* 输入 `q_j`：来自常量模块 (高斯基底均值矩阵)。
* 输出 `tau`：连接至 Memory 模块，并输出到 Workspace。
* 输出 `e2`：连接至 adaptive_update 模块。

**4. adaptive_update (自适应与指令滤波律)**
* 输入 `e2`：来自 controller_fuzzy 模块。
* 输入 `e2_hat`：来自 Int_e2hat 积分器。
* 输入 `eta`、`nu`：分别来自 Int_eta、Int_nu 积分器。
* 输入 `eta_dot`：来自 auv_plant 模块。
* 输入 `q_j`、`M_inv`：分别来自对应的常量模块。
* 输出 `phi_dot`、`e2_hat_dot`：分别连接至 Int_phi、Int_e2hat 积分器。

---

## 二、仿真配置与参数设置清单

### 1. 求解器配置 (Solver Settings)
为保证系统的离散化精度与模糊逻辑更新的稳定性，强烈建议使用以下仿真参数：
* **求解器 (Solver)**：推荐使用定步长 (Fixed-step) 的 `ode4 (Runge-Kutta)`，或变步长 `ode15s`。
* **步长 (Step size)**：务必强制设为 **0.01** (若是 `ode15s`，则将 MaxStep 设为 `0.01`)。
* **仿真时间 (Stop time)**：默认 120 秒，可视收敛情况延长。

### 2. 积分器初始值 (Initial conditions)
* `Int_eta`：`eta_0`
* `Int_nu`：`zeros(6,1)`
* `Int_s1`：`s1_0`
* `Int_phi`：`zeros(17,6)`
* `Int_e2hat`：`zeros(6,1)`
* Memory 模块：`zeros(6,1)`

### 3. 常量与 Workspace 数据记录
* 常量赋值：`tau_E` -> `tau_E_inject`，`xi_hat` -> `xi_hat`，`q_j` -> `q_j`，`M_inv` -> `M_inv`。
* Workspace：所有模块的 **Save format** 务必选为 `Array`。变量命名为：`out_time`、`out_eta`、`out_nu`、`out_tau`。

---

## 三、核心模块代码 (精简模块化版)
*已去除冗余注释，增强代码模块化分布与执行逻辑。*

### 1. auv_plant.m (AUV 动力学方程)
```matlab
function [eta_dot, nu_dot] = auv_plant(eta, nu, tau, tau_E)
    % 1. 加载物理参数
    m = 98.0; W = 961.38; B = 952.56; x_G = 0; y_G = 0; z_G = 0.05;
    X_u_dot = 49.0; Y_v_dot = 49.0; Z_w_dot = 49.0; I_x = 8.0; I_y = 8.0; I_z = 8.0;
    X_uu = -148.0; Y_vv = -148.0; Z_ww = -148.0; 
    K_pp = -180.0; M_qq = -180.0; N_rr = -180.0;
    K_p = -130.0; M_q = -130.0; N_r = -130.0;
    
    phi = eta(4); theta = eta(5); psi = eta(6);
    u = nu(1); v = nu(2); w = nu(3); p = nu(4); q = nu(5); r = nu(6);
    
    % 2. 运动学转换
    J1 = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
          sin(psi)*cos(theta),  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
         -sin(theta),         cos(theta)*sin(phi),                            cos(theta)*cos(phi)];
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi),           -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    eta_dot = blkdiag(J1, J2) * nu;
    
    % 3. 动力学矩阵构建
    M = diag([m+X_u_dot, m+Y_v_dot, m+Z_w_dot, I_x, I_y, I_z]);
    D = -diag([X_uu*abs(u), Y_vv*abs(v), Z_ww*abs(w), K_p+K_pp*abs(p), M_q+M_qq*abs(q), N_r+N_rr*abs(r)]);
    
    C = zeros(6,6);
    C(1,5) = M(3,3)*w; C(1,6) = -M(2,2)*v;
    C(2,4) = -M(3,3)*w; C(2,6) = M(1,1)*u;
    C(3,4) = M(2,2)*v; C(3,5) = -M(1,1)*u;
    C(4,5) = -M(6,6)*r; C(4,6) = M(5,5)*q;
    C(5,4) = M(6,6)*r; C(5,6) = -M(4,4)*p;
    C(6,4) = -M(5,5)*q; C(6,5) = M(4,4)*p;
    C = C - C';
    
    g = [(W-B)*sin(theta);
         -(W-B)*cos(theta)*sin(phi);
         -(W-B)*cos(theta)*cos(phi);
         y_G*W*cos(theta)*cos(phi) - z_G*W*cos(theta)*sin(phi);
         -x_G*W*cos(theta)*cos(phi) - z_G*W*sin(theta);
         x_G*W*cos(theta)*sin(phi) + y_G*W*sin(theta)];
    
    % 4. 计算速度导数
    nu_dot = M \ (tau + tau_E - C*nu - D*nu - g);
end
```

### 2. visual_model.m (视觉投影与雅可比)
```matlab
function [xi, J_xi, s1_dot] = visual_model(eta, nu, s1_current)
    % 1. 状态提取与防除零处理
    z = eta(3); phi = eta(4); theta = eta(5); psi = eta(6);
    Z = abs(z) + 0.01; 
    
    % 2. 特征投影计算
    f_x = 1; f_y = 1; c_x = 0; c_y = 0; 
    x_v = eta(1)/Z; 
    x_u = eta(2)/Z;
    xi = [x_v; x_u; s1_current; phi; theta; psi];
    
    % 3. 图像雅可比矩阵
    a1 = x_v - c_y; a2 = x_u - c_x;
    L_R_Z = [-f_y/Z,  0,       a1/Z,  a1*a2/f_x,            -(f_y^2+a1^2)/f_y, a2*f_y/f_x;
              0,      -f_x/Z,  a2/Z,  (f_x^2+a2^2)/f_x,     -a1*a2/f_y,        -a1*f_x/f_y];
    
    J3 = [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi),           -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    
    J_xi = [L_R_Z;
            (2*s1_current/Z)*J3, zeros(1,3);
            zeros(3,3), J2];
            
    % 4. 面积特征导数
    Z_dot = -J3 * nu(1:3); 
    s1_dot = (-2 * s1_current / Z) * Z_dot;
end
```

### 3. controller_fuzzy.m (模糊控制核心)
```matlab
function [tau, e2] = controller_fuzzy(xi, xi_hat, nu, J_xi, eta, eta_dot, phi_weights, q_j)
    tau = zeros(6, 1);
    e2 = zeros(6, 1);

    k1 = 2; k2 = 100; d_j = 2;
    
    % 1. 误差计算与虚拟控制
    e1 = xi - xi_hat;
    alpha_1 = -pinv(J_xi) * (k1 * e1);
    e2 = nu - alpha_1;

    % 2. 模糊基函数激活
    gamma = [eta; eta_dot; nu];
    S_gamma_unnorm = zeros(17, 1);
    for i = 1:17
        S_gamma_unnorm(i) = exp(-norm(gamma - q_j(:,i))^2 / d_j^2);
    end
    S_gamma = S_gamma_unnorm / (sum(S_gamma_unnorm) + 1e-6);

    % 3. 控制指令合成
    tau = phi_weights' * S_gamma - k2 * e2;
end
```

### 4. adaptive_update.m (自适应与滤波律)
```matlab
function [phi_dot, e2_hat_dot] = adaptive_update(e2, e2_hat, eta, eta_dot, nu, q_j, M_inv)
    phi_dot = zeros(17, 6);
    e2_hat_dot = zeros(6, 1);

    k2 = 100; d_j = 2;
    b1 = 50 * eye(6);
    k3 = diag([90, 90, 5, 1, 1, 1]);

    % 1. 指令滤波补偿误差
    e2_hat_dot = -b1 * (e2_hat - e2);

    % 2. 模糊基函数计算
    gamma = [eta; eta_dot; nu];
    S_gamma_unnorm = zeros(17, 1);
    for i = 1:17
        S_gamma_unnorm(i) = exp(-norm(gamma - q_j(:,i))^2 / d_j^2);
    end
    S_gamma = S_gamma_unnorm / (sum(S_gamma_unnorm) + 1e-6);

    % 3. 自适应权重更新律
    for i = 1:6
        phi_dot(:, i) = k3(i,i) * (e2_hat_dot(i) + k2 * M_inv(i,i) * e2(i)) * S_gamma;
    end
end
```