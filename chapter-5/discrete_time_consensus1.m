% 多智能体系统一致性的控制基础及其应用
% 第5章-二阶多智体系统一致性 -＞ 离散时间系统动态一致性
% Author: Zhao-Jichao
% Date: 2022-08-03
clear
clc

%% Laplacian Matrix
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
u(:,:) = [00 00 00 00]';

%% Time Parameters
tBegin = 0;
tFinal = 20;
dT = 0.01;
times = (tFinal-tBegin)/dT;
t(1,1) = 0;

%% Iteration Calculate
for k=1:times
    % record time
    t(:,k+1) = t(:,k) + dT;
    
    % calculate control inputs
    u(:,k) = [-alpha*L -beta*L] * [p(:,k); v(:,k)];
    
    % update statues
    v(:,k+1) = v(:,k) + dT * u(:,k);
    p(:,k+1) = p(:,k) + dT * v(:,k);
end

%% Draw graphs
figure(1)
plot(t,p(1,:), t,p(2,:), t,p(3,:), t,p(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$p_i$','Interpreter','latex');
legend('$p_1$','$p_2$','$p_3$','$p_4$','Interpreter','latex'); grid on
xlim([0,20])

figure(2)
plot(t,v(1,:), t,v(2,:), t,v(3,:), t,v(4,:), 'linewidth',1.5);
xlabel('$t (s)$','Interpreter','latex');
ylabel('$v_i$','Interpreter','latex');
legend('$v_1$','$v_2$','$v_3$','$v_4$','Interpreter','latex'); grid on
xlim([0,20])
