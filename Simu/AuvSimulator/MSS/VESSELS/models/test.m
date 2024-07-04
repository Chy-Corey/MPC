function [xdot, U] = test(x, ui, Vc, betaVc, w_c)
    % 参数设置
    % 状态矩阵 A (12x12)
    A = zeros(12, 12);
    
    % 假设位置部分的状态变量 (x, y, z)
    A(1, 4) = 1; % x_dot = u
    A(2, 5) = 1; % y_dot = v
    A(3, 6) = 1; % z_dot = w
    
    % 假设速度部分的状态变量 (u, v, w)
    A(4, 4) = -0.1; % u_dot = -0.1 * u
    A(5, 5) = -0.1; % v_dot = -0.1 * v
    A(6, 6) = -0.1; % w_dot = -0.1 * w
    
    % 假设姿态部分的状态变量 (phi, theta, psi)
    A(10, 7) = 1; % phi_dot = p
    A(11, 8) = 1; % theta_dot = q
    A(12, 9) = 1; % psi_dot = r
    
    % 假设角速度部分的状态变量 (p, q, r)
    A(7, 7) = -0.05; % p_dot = -0.05 * p
    A(8, 8) = -0.05; % q_dot = -0.05 * q
    A(9, 9) = -0.05; % r_dot = -0.05 * r
    
    % 控制输入矩阵 B (12x3)
    B = zeros(12, 3);
    B(4, 1) = 1; % 控制输入对 u 的影响
    B(5, 2) = 1; % 控制输入对 v 的影响
    B(6, 3) = 1; % 控制输入对 w 的影响
    B(7, 1) = 0.1; % 控制输入对 p 的影响
    B(8, 2) = 0.1; % 控制输入对 q 的影响
    B(9, 3) = 0.1; % 控制输入对 r 的影响

    % 检查系统的可控性


    % 状态更新方程
    xdot = A * x + B * ui;

    % 计算速度 U
    U = sqrt(x(4)^2 + x(5)^2 + x(6)^2);

    % 可选：考虑洋流影响
    if nargin > 2
        % 计算洋流影响的分量
        u_c = Vc * cos(betaVc);
        v_c = Vc * sin(betaVc);

        % 更新状态变化量，假设洋流只影响速度部分
        xdot(4) = xdot(4) + u_c;
        xdot(5) = xdot(5) + v_c;
        xdot(6) = xdot(6) + w_c;
    end
end
