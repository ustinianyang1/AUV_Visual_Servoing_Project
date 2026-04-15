function [d_hat_x1, d_hat_x2, d_W_out, Phi] = esn_observer(x1, hat_x1, hat_x2, u_esn, Phi_delay, W_out_hat, W_in, W_d, M0, J, tau, K1, K2, rho, kw)
% ESN_OBSERVER 实现基于 Echo State Network 的状态观察器
%
% 输入：
% x1：测量位置矢量 [x, y, z, psi]^T
% hat_x1：估计位置向量
% hat_x2：估计速度向量
% u_esn：ESN输入向量[x1(t)； x1(t-td); x1(t-2td); τ(t)] (16x1)
% Phi_delay：t - td (8x1) 时的 ESN 储存器状态
% W_out_hat：当前ESN输出权重（8x4）
%   W_in: input weights (8x16)
% W_d：储层内部权重（8x8）
% M0：惯性矩阵的逆矩阵或标称惯性矩阵 (4x4)
% J：旋转矩阵（4x4）
% tau：电流控制输入 (4x1)
% K1、K2：观察者增益矩阵 (4x4)
% rho, kw：W_out 的自适应率和泄漏增益
%
% 输出：
% d_hat_x1：估计位置的时间导数
% d_hat_x2：估计速度的时间导数
% d_W_out：输出权重的时间导数
% Phi：当前ESN储层状态

    % 1.更新ESN水库状态Phi
    % 激活函数：高斯函数 sigma(x) = exp(-x.^2)
    internal_activation = W_in * u_esn + W_d * Phi_delay;
    Phi = exp(-(internal_activation.^2));

    % 2. 观察者错误
    tilde_x1 = hat_x1 - x1;

    % 3. 状态观察者动力学
    % d_hat_x1 = -K1 * 波形符_x1 + J * hat_x2
    d_hat_x1 = -K1 * tilde_x1 + J * hat_x2;
    
    % g0 是标称恢复力 - 假设为零或预先补偿
    g0 = zeros(4,1); 
    
    % d_hat_x2 = -M0^{-1}*K2*J^T*tilde_x1 + M0^{-1}[tau - W_out_hat^T*Phi - g0]
    invM0 = inv(M0);
    d_hat_x2 = -invM0 * K2 * J' * tilde_x1 + invM0 * (tau - W_out_hat' * Phi - g0);

    % 4.输出权重W_out_hat的自适应律
    % d_W_out_hat = -rho * ( Phi * tilde_x1^T * J + kw * W_out_hat )
    d_W_out = -rho * ( Phi * tilde_x1' * J + kw * W_out_hat );

end
