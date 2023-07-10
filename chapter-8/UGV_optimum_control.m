% 多智能体系统一致性的控制基础及其应用
% 第8章 - 异构系统的协同控制及最优控制 --＞ 实验验证
% 8.4.2 实验2：多无人车系统的最优编队控制
% Date: 2023-06-06
% Author: Zhao-Jichao
clear
clc

%% 
% Topology of system
L = [1  0 -1
    -1  2 -1
     0 -1  1];

% States
% UGV_i = [p_x_i p_y_i]
UGV_1(:,1) = [10  20]';
UGV_2(:,1) = [10  30]';
UGV_3(:,1) = [10  10]';

% P_X = [p_x_1 p_x_2 p_x_3 p_x_5 p_x_5]
P_X = [UGV_1(1,1)  UGV_2(1,1)  UGV_3(1,1)];
P_Y = [UGV_1(2,1)  UGV_2(2,1)  UGV_3(2,1)];
X_G(:,1) = [P_X  P_Y]';

% Matrices of system
a_G = [0 0; 0 0];
b_G = [1 0; 0 1];
N_G = 3;
A_G = kron(a_G, eye(N_G));
B_G = kron(b_G, eye(N_G));

Q_G = 5*eye(2);
R_G = 1*eye(2);
P_G = care(a_G, b_G, Q_G, R_G);
K_G = pinv(R_G) * (b_G)' * P_G;
k_G = K_G(1,1);

% Formation
d_x_G1 = 0;    d_y_G1 = 0;
d_x_G2 = -10;  d_y_G2 = 10;
d_x_G3 = -10;  d_y_G3 = -10;
d = [d_x_G1  d_x_G2  d_x_G3  d_y_G1  d_y_G2  d_y_G3]';

% Time parameters
tBegin = 0;
tFinal = 5;
dT = 0.05;
times = (tFinal-tBegin)/dT;
t(1,1) = 0;

% Iterations
for i = 1:times
	t(:,i+1) = t(:,i) + dT;
    U_G = -k_G * kron(eye(2), L) * (X_G(:,i) - d) + [10 10 10 0 0 0]';
    dotX_G = A_G * X_G(:,i) + B_G * U_G;
    X_G(:,i+1) = X_G(:,i) + dT * dotX_G;
end

UGV1(1,:) = X_G(1,:);  UGV1(2,:) = X_G(4,:);
UGV2(1,:) = X_G(2,:);  UGV2(2,:) = X_G(5,:);
UGV3(1,:) = X_G(3,:);  UGV3(2,:) = X_G(6,:);

%% Resultes
figure()
plot(UGV1(1,:),UGV1(2,:), '-',  'linewidth',1.5); hold on;
plot(UGV2(1,:),UGV2(2,:), '--', 'linewidth',1.5);
plot(UGV3(1,:),UGV3(2,:), '-.', 'linewidth',1.5);
grid on;

tt1 = 1;
scatter3(UGV1(1,tt1),UGV1(2,tt1),tt1*dT,100); hold on
scatter3(UGV2(1,tt1),UGV2(2,tt1),tt1*dT,100); hold on
scatter3(UGV3(1,tt1),UGV3(2,tt1),tt1*dT,100); hold on
line([UGV1(1,tt1),UGV2(1,tt1)],[UGV1(2,tt1),UGV2(2,tt1)],[tt1*dT,tt1*dT], 'linewidth',0.5);
line([UGV2(1,tt1),UGV3(1,tt1)],[UGV2(2,tt1),UGV3(2,tt1)],[tt1*dT,tt1*dT], 'linewidth',0.5);
line([UGV3(1,tt1),UGV1(1,tt1)],[UGV3(2,tt1),UGV1(2,tt1)],[tt1*dT,tt1*dT], 'linewidth',0.5);
text(UGV3(1,tt1)-1,UGV3(2,tt1)-1,tt1*dT,'t = 0s','Interpreter','latex')

tt1 = 1/dT;
scatter3(UGV1(1,tt1),UGV1(2,tt1),tt1*dT,100); hold on
scatter3(UGV2(1,tt1),UGV2(2,tt1),tt1*dT,100); hold on
scatter3(UGV3(1,tt1),UGV3(2,tt1),tt1*dT,100); hold on
line([UGV1(1,tt1),UGV2(1,tt1)],[UGV1(2,tt1),UGV2(2,tt1)],[tt1*dT,tt1*dT], 'linewidth',0.5);
line([UGV2(1,tt1),UGV3(1,tt1)],[UGV2(2,tt1),UGV3(2,tt1)],[tt1*dT,tt1*dT], 'linewidth',0.5);
line([UGV3(1,tt1),UGV1(1,tt1)],[UGV3(2,tt1),UGV1(2,tt1)],[tt1*dT,tt1*dT], 'linewidth',0.5);
text(UGV3(1,tt1)+1,UGV3(2,tt1)-1,tt1*dT,'t = 1s','Interpreter','latex')

tt2 = 3/dT;
scatter3(UGV1(1,tt2),UGV1(2,tt2),tt2*dT,100); hold on
scatter3(UGV2(1,tt2),UGV2(2,tt2),tt2*dT,100); hold on
scatter3(UGV3(1,tt2),UGV3(2,tt2),tt2*dT,100); hold on
line([UGV1(1,tt2),UGV2(1,tt2)],[UGV1(2,tt2),UGV2(2,tt2)],[tt2*dT,tt2*dT], 'linewidth',0.5);
line([UGV2(1,tt2),UGV3(1,tt2)],[UGV2(2,tt2),UGV3(2,tt2)],[tt2*dT,tt2*dT], 'linewidth',0.5);
line([UGV3(1,tt2),UGV1(1,tt2)],[UGV3(2,tt2),UGV1(2,tt2)],[tt2*dT,tt2*dT], 'linewidth',0.5);
text(UGV3(1,tt2)+1,UGV3(2,tt2)-1,tt2*dT,'t = 3s','Interpreter','latex')

tt3 = 5/dT;
scatter3(UGV1(1,tt3),UGV1(2,tt3),tt3*dT,100); hold on
scatter3(UGV2(1,tt3),UGV2(2,tt3),tt3*dT,100); hold on
scatter3(UGV3(1,tt3),UGV3(2,tt3),tt3*dT,100); hold on
line([UGV1(1,tt3),UGV2(1,tt3)],[UGV1(2,tt3),UGV2(2,tt3)],[tt3*dT,tt3*dT], 'linewidth',0.5);
line([UGV2(1,tt3),UGV3(1,tt3)],[UGV2(2,tt3),UGV3(2,tt3)],[tt3*dT,tt3*dT], 'linewidth',0.5);
line([UGV3(1,tt3),UGV1(1,tt3)],[UGV3(2,tt3),UGV1(2,tt3)],[tt3*dT,tt3*dT], 'linewidth',0.5);
text(UGV3(1,tt3)+1,UGV3(2,tt3)-1,tt3*dT,'t = 5s','Interpreter','latex')
grid on
xlim([0,80]);
ylim([0,40]);
xlabel("$p^x_{gi}$ (s)",'Interpreter','latex', 'FontSize',16);
ylabel("$p^y_{gi}$ (m)",'Interpreter','latex', 'FontSize',16);
legend('无人车1', '无人车2', '无人车3','Interpreter','latex', 'FontSize',14);
