function eta_dot = blimp_kinematics(eta, nu)
% BLIMP_KINEMATICS - 计算 4-DOF 位姿提供的时间导数
% 
% eta：惯性系中的位姿支持[x, y, z, psi]^T。
% nu：身体固定坐标系中的速度支撑[u, v, w, r]^T。
%
% 返回：
% eta_dot：所得的eta时间导数贡献。

    % 提取偏航角
    psi = eta(4);

    % 4-DOF类平面飞行的旋转矩阵 J(eta) + z（无滚转/俯仰）
    J = [cos(psi), -sin(psi), 0, 0;
         sin(psi),  cos(psi), 0, 0;
         0,         0,        1, 0;
         0,         0,        0, 1];

    % 将身体速度转换为惯性坐标率
    eta_dot = J * nu;
end
