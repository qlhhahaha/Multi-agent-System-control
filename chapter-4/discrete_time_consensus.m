% 多智能体系统一致性的控制基础及其应用
% 第4章-一阶多智体系统一致性 -＞ 离散时间系统一致性
% Author: Zhao-Jichao
% Date: 2023-06-06
clear
clc

%% Laplacian Matrix
L = [1  0  0 -1
    -1  1  0  0
    -1 -1  2  0
     0  0 -1  1];

%% Initial States
x_1(:,1) = 20; 
x_2(:,1) = 10; 
x_3(:,1) = 40; 
x_4(:,1) = 00; 

%% Time parameters
tBegin = 0;
tFinal = 20;
epsilon = 0.4;
times = (tFinal-tBegin)/epsilon;
time(1,1) = 0;

%% Iteration Calculate
for i=1:times
    % record time
    time(i+1,1) = time(i,1) + epsilon;
    
    % calculate control inputs
    u_1 = -L(1,1)*(x_1(i,:)-x_1(i,:)) - L(1,2)*(x_2(i,:)-x_1(i,:)) - L(1,3)*(x_3(i,:)-x_1(i,:)) - L(1,4)*(x_4(i,:)-x_1(i,:));
    u_2 = -L(2,1)*(x_1(i,:)-x_2(i,:)) - L(2,2)*(x_2(i,:)-x_2(i,:)) - L(2,3)*(x_3(i,:)-x_2(i,:)) - L(2,4)*(x_4(i,:)-x_2(i,:));
    u_3 = -L(3,1)*(x_1(i,:)-x_3(i,:)) - L(3,2)*(x_2(i,:)-x_3(i,:)) - L(3,3)*(x_3(i,:)-x_3(i,:)) - L(3,4)*(x_4(i,:)-x_3(i,:));
    u_4 = -L(4,1)*(x_1(i,:)-x_4(i,:)) - L(4,2)*(x_2(i,:)-x_4(i,:)) - L(4,3)*(x_3(i,:)-x_4(i,:)) - L(4,4)*(x_4(i,:)-x_4(i,:));
    
    % update statues
    x_1(i+1,1) = x_1(i,1) + epsilon*u_1;
    x_2(i+1,1) = x_2(i,1) + epsilon*u_2;
    x_3(i+1,1) = x_3(i,1) + epsilon*u_3;
    x_4(i+1,1) = x_4(i,1) + epsilon*u_4;
end

%% Draw Graphs
figure()
plot(time,x_1, '-',  'linewidth',1.5); hold on;
plot(time,x_2, '--', 'linewidth',1.5);
plot(time,x_3, '-.', 'linewidth',1.5);
plot(time,x_4, ':',  'linewidth',1.5);
grid on;
xlabel('$t$ (s)','Interpreter','latex', 'FontSize',16);
ylabel('$x_i$','Interpreter','latex', 'FontSize',16);
legend('$x_1$','$x_2$','$x_3$','$x_4$','Interpreter','latex', 'FontSize',14);
