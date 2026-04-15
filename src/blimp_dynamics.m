function nu_dot = blimp_dynamics(nu, tau, tau_d, params)
% BLIMP_DYNAMICS - 4 自由度室内飞艇的非线性动力学
%
% nu：底层框架速度 [u, v, w, r]^T
% tau：控制输入力/力矩 [Tu、Tv、Tw、Tr]^T
% tau_d: 外部干扰动力 [du, dv, dw, dr]^T
% params：由 blimp_params.m 加载的包含质量/响应的结构
%
% 返回：
% nu_dot：身体框架中的高度[u_dot，v_dot，w_dot，r_dot]^T。

    u = nu(1);
    v = nu(2);
    % w = nu(3); % z 速度
    r = nu(4);
    
    % 访问物理常数
    mx = params.mx;
    my = params.my;
    % mz = 参数.mz;
    mR = params.mR;
    xg = params.xg;
    
    M = params.M;
    D = params.D;
    
    % 计算科里奥利和向心矩阵C(nu)
    % C(nu) 解释叉积中的连接速度。
    C = zeros(4, 4);
    % 对于没有横滚和俯仰的标准 4DOF 公式：
    % （基于标准AUV平面动力学方程）
    C(1, 4) = -my * v - mR * xg * r;
    C(2, 4) =  mx * u;
    C(4, 1) =  my * v + mR * xg * r;
    C(4, 2) = -mx * u;
    
    % 方程：M*nu_dot + C(nu)*nu + D*nu = tau + tau_d
    % 为ODE工作器隔离nu_dot：
    % nu_dot = M \ (tau + tau_d - C*nu - D*nu)
    
    rhs = tau + tau_d - C * nu - D * nu;
    
    % 纵向坐标 (nu_dot)
    nu_dot = M \ rhs;
    
end
