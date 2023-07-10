% 多智能体系统一致性的控制基础及其应用
% 第5章-二阶多智体系统一致性 -＞ 连续时间系统编队控制
% Author: Zhao-Jichao
% Date: 2022-08-04
clear
clc

%% Laplacian Matrix
global L alpha beta dx
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

% desired formation states
dp(:,:) = [00 -10 -20 -30]';
dv(:,:) = [00 00 00 00]';
dx(:,:) = [dp' dv']';


%% Time Parameters
tBegin = 0;
tFinal = 20;

%% Calculate ODE Function
In = x;
[t,Out] = ode45(@ctFun, [tBegin, tFinal], In);
p = Out(:, 1:4)';
v = Out(:, 5:8)';

%% Draw Graphs
figure
subplot(2,3,1)
plot(t,p(1,:), t,p(2,:), t,p(3,:), t,p(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$p_i$','Interpreter','latex');
legend('$p_1$','$p_2$','$p_3$','$p_4$','Interpreter','latex'); grid on

subplot(2,3,2)
plot(t,v(1,:), t,v(2,:), t,v(3,:), t,v(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$v_i$','Interpreter','latex');
legend('$v_1$','$v_2$','$v_3$','$v_4$','Interpreter','latex'); grid on

subplot(2,3,3)
plot(t,p(1,:)-p(2,:), t,p(2,:)-p(3,:), t,p(3,:)-p(4,:), t,p(4,:)-p(1,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$p_i-p_j$','Interpreter','latex');
legend('$p_1-p_2$','$p_2-p_3$','$p_3-p_4$','$p_4-p_1$','Interpreter','latex'); grid on

subplot(2,3,4)
plot(t,v(1,:)-v(2,:), t,v(2,:)-v(3,:), t,v(3,:)-v(4,:), t,v(4,:)-v(1,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$v_i-v_j$','Interpreter','latex');
legend('$v_1-v_2$','$v_2-v_3$','$v_3-v_4$','$v_4-v_1$','Interpreter','latex'); grid on

subplot(2,3,5)
plot(t,p(1,:)-dp(1,:), t,p(2,:)-dp(2,:), t,p(3,:)-dp(3,:), t,p(4,:)-dp(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$\bar{p}_i$','Interpreter','latex');
legend('$\bar{p}_1$','$\bar{p}_2$','$\bar{p}_3$','$\bar{p}_4$','Interpreter','latex'); grid on

subplot(2,3,6)
plot(t,v(1,:)-dv(1,:), t,v(2,:)-dv(2,:), t,v(3,:)-dv(3,:), t,v(4,:)-dv(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$\bar{v}_i$','Interpreter','latex');
legend('$\bar{v}_1$','$\bar{v}_2$','$\bar{v}_3$','$\bar{v}_4$','Interpreter','latex'); grid on

%% ODE Function
function out = ctFun(~,In)
    x = In;
    global L alpha beta dx
    out = [zeros(4,4)  eye(4)
          -alpha*L    -beta*L] * (x-dx);
end

