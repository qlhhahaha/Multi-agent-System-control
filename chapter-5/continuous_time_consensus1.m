% 多智能体系统一致性的控制基础及其应用
% 第5章-二阶多智体系统一致性 -＞ 连续时间系统动态一致性
% Author: Zhao-Jichao
% Date: 2022-08-03
clear
clc

%% Laplacian Matrix
global L alpha beta
L = [3 -1 -1 -1
    -1  2 -1  0
    -1 -1  3 -1
    -1  0 -1  2];

% gains
alpha = 1.5;
beta = 1.0;

%% Initial States
p(:,:) = [20 10 40 00]';
v(:,:) = [02 01 04 00]';
x(:,:) = [p' v']';

%% Time Parameters
tBegin = 0;
tFinal = 20;

%% Calculate ODE Function
[t,out] = ode45(@ctFun, [tBegin, tFinal], x);
p = out(:, 1:4)';
v = out(:, 5:8)';

%% Draw Graphs
figure(1)
plot(t,p(1,:), t,p(2,:), t,p(3,:), t,p(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$p_i$','Interpreter','latex');
legend('$p_1$','$p_2$','$p_3$','$p_4$','Interpreter','latex'); grid on

figure(2)
plot(t,v(1,:), t,v(2,:), t,v(3,:), t,v(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$v_i$','Interpreter','latex');
legend('$v_1$','$v_2$','$v_3$','$v_4$','Interpreter','latex'); 
grid on

%% ODE Function
function out = ctFun(~,x)
    global L alpha beta
    out = [zeros(4,4)  eye(4)
          -alpha*L    -beta*L] * x;
end
