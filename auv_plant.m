function [eta_dot, nu_dot] = auv_plant(eta, nu, tau, tau_E)
    % 严格对应方程 (1) 和 (2)
    % 加载参数
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
    
    % 动力学矩阵
    M = diag([m+X_u_dot, m+Y_v_dot, m+Z_w_dot, I_x, I_y, I_z]);
    D = -diag([X_uu*abs(u), Y_vv*abs(v), Z_ww*abs(w), K_p+K_pp*abs(p), M_q+M_qq*abs(q), N_r+N_rr*abs(r)]);
    
    % 简化的向心力矩阵 C (基于对角M矩阵)
    C = zeros(6,6);
    C(1,5) = M(3,3)*w; C(1,6) = -M(2,2)*v;
    C(2,4) = -M(3,3)*w; C(2,6) = M(1,1)*u;
    C(3,4) = M(2,2)*v; C(3,5) = -M(1,1)*u;
    C(4,5) = -M(6,6)*r; C(4,6) = M(5,5)*q;
    C(5,4) = M(6,6)*r; C(5,6) = -M(4,4)*p;
    C(6,4) = -M(5,5)*q; C(6,5) = M(4,4)*p;
    C = C - C';
    
    % 恢复力与力矩 g
    g = [(W-B)*sin(theta);
         -(W-B)*cos(theta)*sin(phi);
         -(W-B)*cos(theta)*cos(phi);
         y_G*W*cos(theta)*cos(phi) - z_G*W*cos(theta)*sin(phi);
         -x_G*W*cos(theta)*cos(phi) - z_G*W*sin(theta);
         x_G*W*cos(theta)*sin(phi) + y_G*W*sin(theta)];
    
    % 速度求导 (对应公式 2)
    nu_dot = M \ (tau + tau_E - C*nu - D*nu - g);
end
