function [tau, alpha_1, e1, e2, phi_dot, S_gamma] = controller_fuzzy(xi, xi_hat, nu, J_xi, eta, eta_dot, phi_weights, e2_hat_dot)
    % 严格对应控制律设计 (Step 1 to Step 3)
    k1 = 2; k2 = 100;
    k3 = diag([90, 90, 5, 1, 1, 1]);
    M_inv = inv(diag([98+49, 98+49, 98+49, 8, 8, 8]));
    
    % 虚拟控制律 (公式 15)
    e1 = xi - xi_hat;
    alpha_1 = -pinv(J_xi) * (k1 * e1); % xi_hat 为常数导数为0
    e2 = nu - alpha_1;
    
    % 模糊逻辑系统 (公式 20)
    gamma = [eta; eta_dot; nu]; % 18维输入
    num_rules = 17; d_j = 2;
    rng(0); q_j = -4 + 8 * rand(18, num_rules); % 还原初始化时同样的随机中心
    
    S_gamma_unnorm = zeros(num_rules, 1);
    for i = 1:num_rules
        S_gamma_unnorm(i) = exp(-norm(gamma - q_j(:,i))^2 / d_j^2);
    end
    S_gamma = S_gamma_unnorm / (sum(S_gamma_unnorm) + 1e-6);
    
    % 实际控制律 (公式 19)
    tau = phi_weights' * S_gamma - k2 * e2;
    
    % 自适应律 (公式 29)
    phi_dot = zeros(num_rules, 6);
    for i = 1:6
        phi_dot(:, i) = k3(i,i) * (e2_hat_dot(i) + k2 * M_inv(i,i) * e2(i)) * S_gamma;
    end
end
