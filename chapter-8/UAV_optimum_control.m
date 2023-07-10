% 多智能体系统一致性的控制基础及其应用
% 第8章 - 异构系统的协同控制及最优控制 --＞ 实验验证
% 8.4.1 实验1：多无人机系统的最优编队控制
% Date: 2023-06-06
% Author: Zhao-Jichao
clear
clc

%% 
% Topology of system
L = [2 -1 -1
    -1  2 -1
    -1 -1  2];

% States
P_X = [25 33 45];
P_Y = [25 10 13];
P_Z = [10 00 15];
V_X = [0.0 0.4 0.2];
V_Y = [0.0 0.8 0.4];
V_Z = [0.0 0.0 0.0];
O_T = [0.0 0.0 0.0];
O_P = [0.0 0.0 0.0];
C_0 = [0.0 0.0 0.0];
O_T_Dot = [0.0 0.0 0.0];
O_P_Dot = [0.0 0.0 0.0];
C_0_Dot = [0.0 0.0 0.0];

X_A(:,1) = [P_X P_Y P_Z V_X V_Y V_Z O_T O_P C_0 O_T_Dot O_P_Dot C_0_Dot]';

% Matrices of system
a_A = kron([0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0], eye(3));
b_A = kron([  0; 0; 0; 1], eye(3));
N_A = 3;
A_A = kron(a_A, eye(N_A));
B_A = kron(b_A, eye(N_A));

% Gains
g1 = 0.1;
g2 = 0.8;
g3 = 4.0;
g4 = 1.5;

Q_A = 5*eye(12);
R_A = 1*eye(3);
P_A = care(a_A, b_A, Q_A, R_A);
K_A = pinv(R_A) * (b_A)' * P_A;
g1 = K_A(1,1);
g2 = K_A(1,4);
g3 = K_A(1,7);
g4 = K_A(1,10);

L_Ad = kron(kron([g1 g2 0 0], eye(3)), L) + kron(kron([0 0 g3 g4], eye(3)), eye(N_A)) ;

% Formation states
d_P_Ai = zeros(36,1);

d_P_Ai(1,1) = 00;     % P_X of UAV1
d_P_Ai(2,1) =-10;     % P_X of UAV2
d_P_Ai(3,1) = 10;     % P_X of UAV3

d_P_Ai(4,1) = 00;     % P_Y of UAV1
d_P_Ai(5,1) =-10;     % P_Y of UAV2
d_P_Ai(6,1) =-10;     % P_Y of UAV3

% Time parameters
tBegin = 0;
tFinal = 100;
dT = 0.01;
times = (tFinal-tBegin)/dT;
t(1,1) = 0;

% Iterations
for i = 1:times
    t(:,i+1) = t(:,i) + dT;
    U_A = -L_Ad * ( X_A(:,i)-d_P_Ai );
    U_A(1,1) = 0;
    U_A(4,1) = 0;
    U_A(7,1) = 0;
    dotX_A = A_A * X_A(:,i) + B_A * U_A;
    X_A(:,i+1) = X_A(:,i) + dT * dotX_A;
end

%% Resultes
figure()
plot(X_A( 1, :), X_A( 4, :), '-',  'linewidth',1.5); hold on;
plot(X_A( 2, :), X_A( 5, :), '--', 'linewidth',1.5);
plot(X_A( 3, :), X_A( 6, :), '-.', 'linewidth',1.5);
ti = tFinal/dT;
line([X_A( 1, ti), X_A( 2, ti)], [X_A( 4, ti), X_A( 5, ti)], 'linewidth',0.5);
line([X_A( 2, ti), X_A( 3, ti)], [X_A( 5, ti), X_A( 6, ti)], 'linewidth',0.5);
line([X_A( 3, ti), X_A( 1, ti)], [X_A( 6, ti), X_A( 4, ti)], 'linewidth',0.5);
scatter(X_A( 1, ti), X_A( 4, ti)); hold on;
scatter(X_A( 2, ti), X_A( 5, ti));
scatter(X_A( 3, ti), X_A( 6, ti));
xlabel("$p^x_i (m)$",'Interpreter','latex', 'FontSize',16);
ylabel("$p^y_i (m)$",'Interpreter','latex', 'FontSize',16);
legend('无人机1', '无人机2', '无人机3','Interpreter','latex', 'FontSize',14);
grid on;
xlim([0,50]);
ylim([0,50]);
