```markdown
# AUV 凸优化自适应模糊视觉伺服控制系统仿真模型文档
*(基于 IEEE TCE 论文的最终稳定复刻版)*

## 一、核心连线逻辑表 (5个积分器 + Memory打断架构)

本架构彻底规避了 Simulink 的“代数环”死锁问题，并通过分离常数输入解决了 Coder 编译维度欠定的报错。

### 模块连接关系

**[模块 1：auv_plant (AUV 本体动力学)]**
- 输入 `tau`       <--- **Memory 模块**的输出 (核心：打断代数环)
- 输入 `tau_E`     <--- 常量模块 (扰动输入 `tau_E_inject`)
- 输入 `eta`       <--- 积分器 1 (`Int_eta`) 的输出
- 输入 `nu`        <--- 积分器 2 (`Int_nu`) 的输出
- 输出 `eta_dot`   ---> 连往 积分器 1、模块 3、模块 4
- 输出 `nu_dot`    ---> 连往 积分器 2

**[模块 2：visual_model (极简视觉混合模型)]**
- 输入 `eta`       <--- 积分器 1 (`Int_eta`) 的输出
- 输入 `nu`        <--- 积分器 2 (`Int_nu`) 的输出
- 输入 `s1_current`<--- 积分器 3 (`Int_s1`) 的输出
- 输出 `xi`        ---> 连往 模块 3
- 输出 `J_xi`      ---> 连往 模块 3
- 输出 `s1_dot`    ---> 连往 积分器 3

**[模块 3：controller_fuzzy (模糊控制器)]**
- 输入 `xi`        <--- 模块 2 的 `xi` 输出
- 输入 `xi_hat`    <--- 常量模块 (目标视觉特征 `xi_hat`)
- 输入 `nu`        <--- 积分器 2 (`Int_nu`) 的输出
- 输入 `J_xi`      <--- 模块 2 的 `J_xi` 输出
- 输入 `eta`       <--- 积分器 1 (`Int_eta`) 的输出
- 输入 `eta_dot`   <--- 模块 1 的 `eta_dot` 输出
- 输入 `phi_weights`<-- 积分器 4 (`Int_phi`) 的输出
- 输入 `q_j`       <--- 常量模块 (高斯基底均值矩阵 `q_j`)
- 输出 `tau`       ---> 连往 **Memory 模块** 的输入 (再连回模块1) 和 To Workspace
- 输出 `e2`        ---> 连往 模块 4

**[模块 4：adaptive_update (自适应与指令滤波律)]**
- 输入 `e2`        <--- 模块 3 的 `e2` 输出
- 输入 `e2_hat`    <--- 积分器 5 (`Int_e2hat`) 的输出
- 输入 `eta`       <--- 积分器 1 (`Int_eta`) 的输出
- 输入 `eta_dot`   <--- 模块 1 的 `eta_dot` 输出
- 输入 `nu`        <--- 积分器 2 (`Int_nu`) 的输出
- 输入 `q_j`       <--- 常量模块 (高斯基底均值矩阵 `q_j`)
- 输入 `M_inv`     <--- 常量模块 (惯性矩阵的逆 `M_inv`)
- 输出 `phi_dot`   ---> 连往 积分器 4
- 输出 `e2_hat_dot`---> 连往 积分器 5

### 5个积分器前端输入
- `Int_eta` 输入   <--- 模块 1 输出的 `eta_dot`
- `Int_nu` 输入    <--- 模块 1 输出的 `nu_dot`
- `Int_s1` 输入    <--- 模块 2 输出的 `s1_dot`
- `Int_phi` 输入   <--- 模块 4 输出的 `phi_dot`
- `Int_e2hat` 输入 <--- 模块 4 输出的 `e2_hat_dot`

---

## 二、初始值与参数设置清单

### 1. 积分器 (Integrator) & Memory -> Initial condition
- 积分器 1 (`Int_eta`)    填入: `eta_0`
- 积分器 2 (`Int_nu`)     填入: `zeros(6,1)`
- 积分器 3 (`Int_s1`)     填入: `s1_0`
- 积分器 4 (`Int_phi`)    填入: `zeros(17,6)`
- 积分器 5 (`Int_e2hat`)  填入: `zeros(6,1)`
- **Memory 模块** 填入: `zeros(6,1)`

### 2. 常量模块 (Constant) -> Constant value
- 常量模块 `tau_E`        填入: `tau_E_inject`
- 常量模块 `xi_hat`       填入: `xi_hat`
- 常量模块 `q_j`          填入: `q_j`
- 常量模块 `M_inv`        填入: `M_inv`

### 3. 输出到工作区 (To Workspace) -> Save format 务必选 Array
- 时钟 Clock 模块         -> Variable name: `out_time`
- 记录 `eta` 模块         -> Variable name: `out_eta`
- 记录 `nu` 模块          -> Variable name: `out_nu`
- 记录 `tau` 模块         -> Variable name: `out_tau`

---

## 三、核心模块代码 (MATLAB Function 完整版)

### 模块 1：auv_plant.m (AUV 本体动力学)
```matlab
function [eta_dot, nu_dot] = auv_plant(eta, nu, tau, tau_E)
    % 提取从工作区加载的全局参数
    m = 98.0; W = 961.38; B = 952.56; x_G = 0; y_G = 0; z_G = 0.05;
    X_u_dot = 49.0; Y_v_dot = 49.0; Z_w_dot = 49.0; I_x = 8.0; I_y = 8.0; I_z = 8.0;
    X_uu = -148.0; Y_vv = -148.0; Z_ww = -148.0; K_pp = -180.0; M_qq = -180.0; N_rr = -180.0;
    K_p = -130.0; M_q = -130.0; N_r = -130.0;
    
    phi = eta(4); theta = eta(5); psi = eta(6);
    u = nu(1); v = nu(2); w = nu(3); p = nu(4); q = nu(5); r = nu(6);
    
    % 运动学矩阵 J(eta_2)
    J1 = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
          sin(psi)*cos(theta),  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
         -sin(theta),         cos(theta)*sin(phi),                            cos(theta)*cos(phi)];
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi),           -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    J_eta = blkdiag(J1, J2);
    eta_dot = J_eta * nu;
    
    % 动力学矩阵 M, D, C, g
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
    
    nu_dot = M \ (tau + tau_E - C*nu - D*nu - g);
end

```

### 模块 2：visual_model.m (极简稳定版视觉模型)

```matlab
function [xi, J_xi, s1_dot] = visual_model(eta, nu, s1_current)
    z = eta(3); phi = eta(4); theta = eta(5); psi = eta(6);
    Z = abs(z) + 0.01; % 避免除零
    
    f_x = 1; f_y = 1; c_x = 0; c_y = 0; % 归一化极简内参
    x_v = eta(1)/Z; 
    x_u = eta(2)/Z;
    xi = [x_v; x_u; s1_current; phi; theta; psi];
    
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
            
    Z_dot = -J3 * nu(1:3); 
    s1_dot = (-2 * s1_current / Z) * Z_dot;
end

```

### 模块 3：controller_fuzzy.m (模糊控制器)

```matlab
function [tau, e2] = controller_fuzzy(xi, xi_hat, nu, J_xi, eta, eta_dot, phi_weights, q_j)
    % 强制定义输出维度，规避编译报错
    tau = zeros(6, 1);
    e2 = zeros(6, 1);

    k1 = 2; 
    k2 = 100; 
    d_j = 2;
    
    % 虚拟控制律与误差计算
    e1 = xi - xi_hat;
    alpha_1 = -pinv(J_xi) * (k1 * e1);
    e2 = nu - alpha_1;

    % 模糊逻辑系统基函数
    gamma = [eta; eta_dot; nu];
    S_gamma_unnorm = zeros(17, 1);
    for i = 1:17
        S_gamma_unnorm(i) = exp(-norm(gamma - q_j(:,i))^2 / d_j^2);
    end
    S_gamma = S_gamma_unnorm / (sum(S_gamma_unnorm) + 1e-6);

    % 实际控制指令 (推力)
    tau = phi_weights' * S_gamma - k2 * e2;
end

```

### 模块 4：adaptive_update.m (自适应与指令滤波器)

```matlab
function [phi_dot, e2_hat_dot] = adaptive_update(e2, e2_hat, eta, eta_dot, nu, q_j, M_inv)
    % 强制定义输出维度
    phi_dot = zeros(17, 6);
    e2_hat_dot = zeros(6, 1);

    k2 = 100; 
    d_j = 2;
    b1 = 50 * eye(6);
    k3 = diag([90, 90, 5, 1, 1, 1]);

    % 指令滤波器
    e2_hat_dot = -b1 * (e2_hat - e2);

    % 重新计算高斯基底
    gamma = [eta; eta_dot; nu];
    S_gamma_unnorm = zeros(17, 1);
    for i = 1:17
        S_gamma_unnorm(i) = exp(-norm(gamma - q_j(:,i))^2 / d_j^2);
    end
    S_gamma = S_gamma_unnorm / (sum(S_gamma_unnorm) + 1e-6);

    % 计算权重导数 (自适应律)
    for i = 1:6
        phi_dot(:, i) = k3(i,i) * (e2_hat_dot(i) + k2 * M_inv(i,i) * e2(i)) * S_gamma;
    end
end

```