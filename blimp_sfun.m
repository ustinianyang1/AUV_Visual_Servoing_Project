function blimp_sfun(block)
    setup(block);
end

function setup(block)
    block.NumInputPorts  = 3;
    block.NumOutputPorts = 2;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    block.InputPort(1).Dimensions = 4;
    block.InputPort(2).Dimensions = 4;
    block.InputPort(3).Dimensions = 4;
    block.OutputPort(1).Dimensions = 4;
    block.OutputPort(2).Dimensions = 4;
    block.NumDialogPrms = 2;
    block.SampleTimes = [0.01 0];
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
end

function DoPostPropSetup(block)
    block.NumDworks = 8;
    sizes = [4, 4, 32, 4, 4, 8, 4, 4];
    % 【修复关键】必须给每一个 Dwork 向量起名字，否则报错
    names = {'x1_hat','x2_hat','W_out','x1_prev1','x1_prev2','Phi_prev','tau_prev','vc_prev'};
    for i = 1:8
        block.Dwork(i).Name = names{i};
        block.Dwork(i).Dimensions = sizes(i);
        block.Dwork(i).DatatypeID = 0;
        block.Dwork(i).Complexity = 'Real';
    end
end

function InitConditions(block)
    block.Dwork(1).Data = [1.3; -0.1; -0.06; -0.03];
    block.Dwork(2).Data = zeros(4,1);
    block.Dwork(3).Data = zeros(32,1);
    block.Dwork(4).Data = [1.3; -0.1; -0.06; -0.03];
    block.Dwork(5).Data = [1.3; -0.1; -0.06; -0.03];
    block.Dwork(6).Data = zeros(8,1);
    block.Dwork(7).Data = zeros(4,1);
    block.Dwork(8).Data = zeros(4,1);
end

function [tau, x2_hat, dx1_hat, dx2_hat, dW_out, Phi, vc] = compute_core(block)
    t = block.CurrentTime; dt = 0.01;
    x1 = block.InputPort(1).Data; x1d = block.InputPort(2).Data; x1d_dot = block.InputPort(3).Data;
    x1_hat = block.Dwork(1).Data; x2_hat = block.Dwork(2).Data;
    W_out = reshape(block.Dwork(3).Data, [8, 4]);
    Phi_prev = block.Dwork(6).Data; tau_prev = block.Dwork(7).Data; vc_prev = block.Dwork(8).Data;
    W_in = block.DialogPrm(1).Data; W_d = block.DialogPrm(2).Data;
    
    m_R = 0.884; m_Ax = 0.585; m_Ay = 0.585; m_Az = 0.607; I_RBz = 0.039; I_Az = 0.012; x_g = 0.05;
    M0 = [m_R+m_Ax 0 0 0; 0 m_R+m_Ay 0 m_R*x_g; 0 0 m_R+m_Az 0; 0 m_R*x_g 0 I_RBz+I_Az];
    
    u = [x1; block.Dwork(4).Data; block.Dwork(5).Data; tau_prev];
    Phi = tanh(W_in * u + W_d * Phi_prev);
    psi = x1(4); J = [cos(psi) -sin(psi) 0 0; sin(psi) cos(psi) 0 0; 0 0 1 0; 0 0 0 1];
    
    kl = [0.5*exp(-0.4*t)+0.1; 0.5*exp(-0.4*t)+0.1; 0.3*exp(-0.4*t)+0.15; 0.1*exp(-0.3*t)+0.015];
    kh = [0.4*exp(-0.4*t)+0.2; 0.4*exp(-0.4*t)+0.2; 0.3*exp(-0.7*t)+0.1;  0.1*exp(-0.2*t)+0.015];
    kl_dot = [-0.2*exp(-0.4*t); -0.2*exp(-0.4*t); -0.12*exp(-0.4*t); -0.03*exp(-0.3*t)];
    kh_dot = [-0.16*exp(-0.4*t); -0.16*exp(-0.4*t); -0.21*exp(-0.7*t); -0.02*exp(-0.2*t)];
    
    z1 = x1 - x1d; Lambda = zeros(4,1); Psi_vec = zeros(4,1);
    K3 = diag([0.5, 0.5, 1.1, 0.5]);
    for i = 1:4
        z1i = z1(i); q = double(z1i > 0);
        if q==1, kb = kh(i); kbd = kh_dot(i); else, kb = kl(i); kbd = kl_dot(i); end
        ratio = min(z1i^2 / kb^2, 0.95);
        if abs(z1i) < 1e-5, Lambda(i) = 0; else
            Lambda(i) = -K3(i,i)*kb^2*sin(pi*ratio)/(2*pi*z1i) + (kbd/kb)*z1i;
        end
        Psi_vec(i) = z1i / (cos(pi*ratio/2)^2 + 1e-4);
    end
    
    vc = J' * (Lambda + x1d_dot);
    vc_dot = max(min((vc - vc_prev)/dt, 50), -50);
    z2_hat = x2_hat - vc;
    tau_raw = W_out' * Phi + M0 * vc_dot - J' * Psi_vec - diag([1.4, 1.4, 2.6, 1.2]) * z2_hat;
    tau = max(min(tau_raw, 10), -10);
    
    x1_tilde = x1_hat - x1;
    dx1_hat = -diag([1.2, 1.2, 2.0, 0.8]) * x1_tilde + J * x2_hat;
    dx2_hat = M0 \ (-diag([1.8, 1.8, 3.0, 1.4])*(J'*x1_tilde) + (tau - W_out'*Phi));
    dW_out = -200 * (Phi * x1_tilde' * J + 0.1 * W_out);
end

function Outputs(block)
    [tau, x2_hat, ~, ~, ~, ~, ~] = compute_core(block);
    block.OutputPort(1).Data = tau;
    block.OutputPort(2).Data = x2_hat;
end

function Update(block)
    [~, ~, dx1_hat, dx2_hat, dW_out, Phi, vc] = compute_core(block);
    dt = 0.01;
    block.Dwork(1).Data = block.Dwork(1).Data + dt * dx1_hat;
    block.Dwork(2).Data = block.Dwork(2).Data + dt * dx2_hat;
    W_next = block.Dwork(3).Data + dt * dW_out(:);
    block.Dwork(3).Data = max(min(W_next, 50), -50);
    block.Dwork(5).Data = block.Dwork(4).Data;
    block.Dwork(4).Data = block.InputPort(1).Data;
    block.Dwork(6).Data = Phi;
    block.Dwork(7).Data = block.OutputPort(1).Data;
    block.Dwork(8).Data = vc;
end
