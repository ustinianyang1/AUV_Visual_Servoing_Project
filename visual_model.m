function [xi, J_xi] = visual_model(eta, nu, s1_current)
    % 对应公式 (8) 和 (11)
    % 这里假设摄像机内参以简化运算，Z 近似为 -z (因为 h=0)
    z = eta(3); phi = eta(4); theta = eta(5); psi = eta(6);
    Z = abs(z) + 0.01; % 避免除零
    
    f_x = 1; f_y = 1; c_x = 0; c_y = 0; % 假定归一化平面
    x_v = eta(1)/Z; % 简化投影
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
end
