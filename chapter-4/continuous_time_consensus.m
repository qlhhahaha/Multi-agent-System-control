% 多智能体系统一致性的控制基础及其应用
% 第4章-一阶多智体系统一致性 -＞ 连续时间系统一致性
% Author: Zhao-Jichao
% Date: 2023-06-06
clear
clc

%% Laplacian Matrix
global L
L = [1  0  0 -1
    -1  1  0  0
    -1 -1  2  0
    0  0 -1  1];

%% Initial States
X0 = [20 10 40 00];

%% Time Parameters
tBegin = 0;
tFinal = 10;

%% Calculate ODE Function
[t,out] = ode45(@ctFun, [tBegin, tFinal], X0);

%% Draw Graphs
figure()
hold on;
plot(t,out( :, 1), '-',  'linewidth',1.5); 
plot(t,out( :, 2), '--', 'linewidth',1.5);
plot(t,out( :, 3), '-.', 'linewidth',1.5);
plot(t,out( :, 4), ':',  'linewidth',1.5);
grid on;
xlabel('$t$ (s)','Interpreter','latex', 'FontSize',16);
ylabel('$x_i$','Interpreter','latex', 'FontSize',16);
legend('$x_1$','$x_2$','$x_3$','$x_4$','Interpreter','latex', 'FontSize',14);

%% ODE Function
function out = ctFun(~, X)
    global L
    dX = -L * X;
    out = dX;
end
