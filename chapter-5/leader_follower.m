% 多智能体系统一致性的控制基础及其应用
% 第5章-二阶多智体系统一致性 -＞ 领航跟随系统一致性动态领航者
% Author: Zhao-Jichao
% Date: 2022-08-04
clear
clc

%% Laplacian Matrix
global L Ll
L = [3 -1 -1 -1
    -1  2 -1  0
    -1 -1  3 -1
    -1  0 -1  2];
Ll= diag([1  1  0  0]);

%% Initial States
p(:,:) = [20 10 40 00]';
v(:,:) = [02 01 04 00]';
x(:,:) = [p' v']';

p_0(:,:) = 20;
v_0(:,:) = 2;
x_0(:,:) = [p_0' v_0']';

%% Gains parameters
global alpha beta
alpha = 1.5;
beta = 1.0;

%% Time Parameters
tBegin = 0;
tFinal = 20;
tspan = [tBegin, tFinal];

%% Calculate ODE Function
In = [x_0'  x']';
out = ode23(@ctFun, tspan, In);
% assigned state variables
t   = out.x;
p_0 = out.y(1,:);
v_0 = out.y(2,:);
p   = out.y(3:6,:);
v   = out.y(7:10,:);

%% Draw Graphs
figure(1)
plot(t,p_0, t,p(1,:), t,p(2,:), t,p(3,:), t,p(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$p_i$','Interpreter','latex');
legend('$p_0$','$p_1$','$p_2$','$p_3$','$p_4$','Interpreter','latex'); grid on

figure(2)
plot(t,v_0, t,v(1,:), t,v(2,:), t,v(3,:), t,v(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$v_i$','Interpreter','latex');
legend('$v_0$','$v_1$','$v_2$','$v_3$','$v_4$','Interpreter','latex'); grid on

%% ODE Function
function out = ctFun(~,In)
    global L Ll alpha beta
    % leader's states
    p_0  = In(1);
    v_0  = In(2);
    a_0  = 0.5;
    dp_0 = v_0;
    dv_0 = a_0;
    
    % followers' states
    p  = In(3:6);
    v  = In(7:10);
    u  = [-alpha*(L+Ll)  -beta*(L+Ll)]...
       * [p - p_0;        v - v_0]...
       + ones(4,1)*a_0;
    dp = v;
    dv = u;
    
    % output 
    out = [dp_0
           dv_0
           dp
           dv];
end
