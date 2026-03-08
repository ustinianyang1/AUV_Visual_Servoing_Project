function Plot_Results(simOut)
    % 处理并绘制 Simulink 仿真结果
    t = squeeze(simOut.out_time);
    eta = squeeze(simOut.out_eta);
    v = squeeze(simOut.out_nu);
    tau = squeeze(simOut.out_tau);
    if size(eta, 2) ~= 6; eta = eta'; end
    if size(v, 2) ~= 6; v = v'; end
    if size(tau, 2) ~= 6; tau = tau'; end

    eta_d = [-0.04; 3.52; 3.45; 0; 0; -1.87]; % 目标位姿

    % 图3：位姿响应
    figure('Name', 'Pose Response', 'Position', [100, 100, 1000, 600]);
    titles_eta = {'(a) x (m)', '(b) y (m)', '(c) z (m)', '(d) \phi (rad)', '(e) \theta (rad)', '(f) \psi (rad)'};
    for i = 1:6
        subplot(2, 3, i);
        plot(t, eta(:, i), 'b', 'LineWidth', 1.5); hold on;
        if i == 3 || i == 6
            plot(t, eta_d(i)*ones(size(t)), 'k--', 'LineWidth', 1);
        elseif i == 4 || i == 5
            plot(t, zeros(size(t)), 'k--', 'LineWidth', 1);
        end
        title(titles_eta{i}); xlabel('time (s)'); grid on; xlim([0, 120]);
    end

    % 图4：速度响应
    figure('Name', 'Velocity Response', 'Position', [150, 150, 1000, 600]);
    titles_v = {'(a) v_u (m/s)', '(b) v_v (m/s)', '(c) v_w (m/s)', '(d) v_p (rad/s)', '(e) v_q (rad/s)', '(f) v_r (rad/s)'};
    for i = 1:6
        subplot(2, 3, i);
        plot(t, v(:, i), 'b', 'LineWidth', 1.5); hold on;
        plot(t, zeros(size(t)), 'k--', 'LineWidth', 1);
        title(titles_v{i}); xlabel('time (s)'); grid on; xlim([0, 120]);
    end

    % 图5：控制量响应
    figure('Name', 'Control Input', 'Position', [200, 200, 1000, 600]);
    titles_tau = {'(a) \tau_1 (N)', '(b) \tau_2 (N)', '(c) \tau_3 (N)', '(d) \tau_4 (N\cdot m)', '(e) \tau_5 (N\cdot m)', '(f) \tau_6 (N\cdot m)'};
    for i = 1:6
        subplot(2, 3, i);
        plot(t, tau(:, i), 'b', 'LineWidth', 1.5);
        title(titles_tau{i}); xlabel('time (s)'); grid on; xlim([0, 120]);
    end

    % 计算稳态误差
    idx_steady = find(t >= 110);
    if ~isempty(idx_steady)
        MAE_eta = mean(abs(eta(idx_steady, :) - [eta(1,1:2), eta_d(3:6)']));
        MAE_v = mean(abs(v(idx_steady, :)));
        disp('【位姿稳态绝对误差 [x, y, z, phi, theta, psi]】:');
        fprintf('%.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', MAE_eta);
        disp('【速度稳态绝对误差 [u, v, w, p, q, r]】:');
        fprintf('%.4f  %.4f  %.4f  %.4f  %.4f  %.4f\n', MAE_v);
    end
end
