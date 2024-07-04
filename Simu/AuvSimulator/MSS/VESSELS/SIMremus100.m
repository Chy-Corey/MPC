function SIMremus100()
% 定义仿真参数
T = 20; % 仿真时间
dt = 0.05; % 时间步长
N = 5; % 预测时域长度
Q = diag([10, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]); % 状态误差权重矩阵
R = diag([1, 1, 0]); % 控制输入权重矩阵
Vc = 0; betaVc = 0; w_c = 0; % 无洋流

% 定义初始状态
x0 = zeros(12, 1); % 初始状态
x0(1) = 0;
x0(2) = 0;
x0(3) = 0;
% 定义控制输入的上下限
lb_single = [-deg2rad(20); -deg2rad(20); 0]; % 单个控制输入的下限
ub_single = [deg2rad(20); deg2rad(20); 1525]; % 单个控制输入的上限
lb = repmat(lb_single, N, 1); % 扩展到整个预测时域
ub = repmat(ub_single, N, 1); % 扩展到整个预测时域
% 定义控制输入变化速率的上下限
delta_u_min = [-deg2rad(5); -deg2rad(5); -10]; % 控制输入变化速率的下限
delta_u_max = [deg2rad(5); deg2rad(5); 10]; % 控制输入变化速率的上限
% 优化选项
options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp', 'MaxIterations', 1000);

% 仿真
x = x0; % 当前状态
X = x0'; % 保存状态的矩阵
U = []; % 保存控制输入的矩阵

% 初始猜测不为零
u0 = repmat([0; 0; 1000], N, 1); % 控制输入的初始猜测
for k = 1:T/dt
    % 定义当前步的参考轨迹
    t_current = (k-1) * dt;
    x_ref = reference_trajectory(t_current, N, dt);
    % 定义代价函数
    cost_function = @(u) mpc_cost_function(u, x, x_ref, N, Q, R, dt, Vc, betaVc, w_c);
    
    % 定义约束
    constraints = @(u) mpc_constraints(u, N, delta_u_min, delta_u_max);
    
    % 使用fmincon求解优化问题
    [u_opt, ~] = fmincon(cost_function, u0, [], [], [], [], lb, ub, constraints, options);
    
    % 应用第一个控制输入
    u_k = u_opt(1:3);
    % 更新状态
    [x_dot, ~] = remus100(x, u_k, Vc, betaVc, w_c);
    x = x + x_dot * dt;
    % 保存状态和控制输入
    X = [X; x'];
    U = [U; u_k'];
    
    % 更新初始猜测
    u0 = [u_opt(4:end); u_opt(end-2:end)];
    disp('Position:')
    disp(x');
end

% 绘图
figure;
subplot(3,1,1);
plot(0:dt:T, X(:, 7), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('x (m)');
title('Position Tracking');

subplot(3,1,2);
plot(0:dt:T, X(:, 8), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('y (m)');

subplot(3,1,3);
plot(0:dt:T, X(:, 9), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('z (m)');

figure;
plot(0:dt:T-dt, U(:, 1), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T-dt, U(:, 2), 'g', 'LineWidth', 2);
plot(0:dt:T-dt, U(:, 3), 'b', 'LineWidth', 2);
legend('delta_r', 'delta_s', 'n');
xlabel('Time (s)');
ylabel('Control Inputs');
title('Control Inputs');

figure;
subplot(2,2,1);
plot(0:dt:T, X(:, 1), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('u');
title('Position Tracking');

subplot(2,2,2);
plot(0:dt:T, X(:, 2), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('v');

subplot(2,2,3);
plot(0:dt:T, X(:, 3), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('w');
subplot(2,2,4);
plot(0:dt:T, X(:, 6), 'r', 'LineWidth', 2); hold on;
plot(0:dt:T, zeros(1, T/dt+1), 'b--', 'LineWidth', 2); % 参考轨迹
legend('AUV Position', 'Reference');
xlabel('Time (s)');
ylabel('r');
end


function x_ref = reference_trajectory(t, N, dt)
    x_ref = zeros(12, N);
    for i = 1:N
        t_future = t + (i-1) * dt;
        x_ref(1, i) = 2; % 假设x方向的参考轨迹是2倍的时间
        x_ref(2, i) = 0.2;
        x_ref(5, i) = 0.2;
        x_ref(6, i) = 0.2;
        % y和z方向保持为0，其他状态也保持为0
    end
end

function J = mpc_cost_function(u, x, x_ref, N, Q, R, dt, Vc, betaVc, w_c)
    J = 0;
    u = reshape(u, 3, N);
    for k = 1:N
        [x_dot, ~] = remus100(x, u(:, k), Vc, betaVc, w_c);
        x = x + x_dot * dt;
        J = J + (x - x_ref(:, k))' * Q * (x - x_ref(:, k)) + u(:, k)' * R * u(:, k);
    end
end

function [c, ceq] = mpc_constraints(u, N, delta_u_min, delta_u_max)
    c = []; % 初始化不等式约束为空
    ceq = []; % 初始化等式约束为空
    u = reshape(u, 3, N); % 将优化变量 u 重新整形为 3xN 矩阵，每列对应一个时间步长的控制输入

    % 对控制输入变化速率的约束
    for k = 1:N-1
        delta_u = u(:, k+1) - u(:, k);
        c = [c; delta_u - delta_u_max]; % 上限约束
        c = [c; delta_u_min - delta_u]; % 下限约束
    end
end

