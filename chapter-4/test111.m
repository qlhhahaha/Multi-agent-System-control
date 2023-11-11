% DGD仿真

num_nodes = 5;  % 节点数量

% 有向图
adjacency_matrix_dir = [ 1  0 -1  0  0;
                        -1  2  0  0 -1;
                        -1 -1  2  0  0;
                         0  0 -1  1  0;
                         0  0  0 -1  1];  

% 无向图
adjacency_matrix_undir = [ 2 -1 -1  0  0;
                          -1  2  0  0 -1;
                          -1  0  2 -1  0;
                           0  0 -1  2 -1;
                           0 -1  0 -1  2];  


% 初始化节点参数和本地梯度
%node_params = rand(num_nodes, 1);  % 初始参数
node_params = [0.1 0.2 0.3 0.4 0.5]';
local_gradients = 0;

% 设置全局目标函数（示例中为简单的平方和）
global_objective = @(x) sum(x.^2);

num_iterations = 1000;
consensus_ratio = 0.01;
fixed_step_size = 0.005;
diminish_step_size = 0.005;

% 创建用于存储可视化数据的数组
param_history = zeros(num_iterations, num_nodes);

iteration_converge = 0;
difference = 0;

for iteration = 1:num_iterations
    for node = 1:num_nodes
        local_gradients = 2 * node_params(node);
        
        % 一致性部分
        for neighbor = 1:num_nodes
            if adjacency_matrix_dir(node, neighbor) == -1
                node_params(node) = node_params(node) - consensus_ratio * (node_params(node) - node_params(neighbor));
            end
        end

        % 梯度下降部分
        diminish_step_size = -0.0045/1000*iteration + 0.005;
        node_params(node) = node_params(node) - fixed_step_size * local_gradients;
    end
    
    % 存储参数历史
    param_history(iteration, :) = node_params;

    % 记录收敛所需迭代次数
    for node = 2:num_nodes
        difference = difference + abs(node_params(1)-node_params(node));
    end
    
    if  iteration_converge == 0 && difference < 0.001
        iteration_converge = iteration;
    end
    difference = 0;
end

% 输出最终的全局最优解
global_minimizer = node_params;
global_minimum = global_objective(global_minimizer);
fprintf('全局最优解：\n');
fprintf('%2f\n', global_minimizer);
fprintf('全局最小值：%f\n', global_minimum);
fprintf('收敛的迭代次数：%d\n', iteration_converge);


figure;
hold on;
for node = 1:num_nodes
    plot(1:num_iterations, param_history(:, node), 'LineWidth', 2, 'DisplayName', sprintf('节点 %d', node));
end
xlabel('迭代次数');
ylabel('节点参数');
title('节点参数随时间的变化');
legend('Location', 'Best');
grid on;
hold off;
