% 多智能体系统一致性的控制基础及其应用
% 第4章-一阶多智体系统一致性 -＞ 连续时间含时延系统一致性
% Author: Zhao-Jichao
% Date: 2022-07-14
clear
clc

%% Laplacian Matrix
global L X0
L = [3 -1 -1 -1
    -1  2 -1  0
    -1 -1  2  0
    -1 -1 -1  3];

%% Initial States
X0 = [20 10 40 00]';

%% Time Parameters
tBegin = 0;
tFinal = 20;
tspan = [tBegin, tFinal];

%% Calculate DDE Function
% Define time-delay
tau = pi/2/max(eig(L)) * 0.8;
out = dde23(@ctFun, tau, @history, tspan);
t = out.x;
X = out.y;

%% Draw Graphs
plot(t,X(1,:), t,X(2,:), t,X(3,:), t,X(4,:), 'linewidth',1.5);
legend('$x_1$','$x_2$','$x_3$','$x_4$', 'Interpreter','latex'); 
grid on
xlabel('$t(s)$', 'Interpreter','latex');
ylabel('$x$', 'Interpreter','latex');

%% DDE Function
function out = ctFun(~,~,TD)
    x_TD = TD;
    global L
    dX = -L * x_TD;
    out = dX;
end

function X_his = history(~)
    global X0
    X_his = X0;
end
