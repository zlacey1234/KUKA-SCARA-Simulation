%% Optimization Example of KUKA SCARA

%%
% Clear the Workspace
clear 
close all 
clc

path_to_workspace = updatePaths();

path_to_CAD = [path_to_workspace, '\KUKA_CAD'];

% Load the SCARA Model
mdl_scara_syms

% Define the symbolic variables for Joints and assume they are 
% real-numbers (not complex-numbers)
th1 = sym('th1', 'real');
th2 = sym('th2', 'real');
th3 = sym('th3', 'real');
d4 = sym('d4', 'real');

qrt_sym_test = [th1 th2 th3 d4];

%% Testing the Jacobian Velocity Propagation Method (Symbolically)
J_velocity_propagation = ...
    jacobian_velocity_propagation(scara, qrt_sym_test, 'rad', 'symbolic');

% % Equivalently 
% J_velocity_propagation = jacobian_velocity_propagation_sym(scara)

%% Testing the Jacobian Force Propagation Method (Symbolically)
J_force_propagation = ...
    jacobian_force_propagation(scara, qrt_sym_test, 'rad', 'symbolic');

% % Equivalently 
% J_force_propagation = jacobian_force_propagation_sym(scara)

%% Testing the Jacobian Explicit Method (Symbolically)
J_explicit = jacobian_explicit(scara, qrt_sym_test, 'rad', 'symbolic');

% % Equivalently
% J_explicit = jacobian_explicit_sym(scara)

%% Display the Kinematic Jacobian Results
disp('Kinematic Jacobian: Velocity Propagation Method');
disp(J_velocity_propagation)

disp('Kinematic Jacobian: Force Propagation Method');
disp(J_force_propagation)

disp('Kinematic Jacobian: Explicit Method');
disp(J_explicit)

%% Singularities
J_reduced = J_explicit;

J_reduced(4:5, :) = [];

DET = simplify(det(J_reduced));

singularities = solve(DET);

%% Optimization Portion Begins (HERE)

%% Known Parameters

% Length of the Tool from Wrist is 170 mm.
Ltool = 0.170;

L1 = 0.400; 

T_wrist_tool_value = [1  0  0  0;...
                      0  1  0  0;...
                      0  0  1  -Ltool;...
                      0  0  0  1];
                  
find_valid_link_pair = 0;

%% Workspace of the PCB/Feeder 

% 2D XY Workspace w.r.t the Base 
x_lower = 0.200;
x_upper = 0.340;

y_lower = -0.050;
y_upper = 0.050;

x_span = x_lower:0.010:x_upper;
y_span = y_lower:0.010:y_upper;

num_x_sample_points = numel(x_span);
num_y_sample_points = numel(y_span);

tolerance = 1e-6;

number_workspace_sample_points = ...
    num_x_sample_points * num_y_sample_points;

xy_sample_points = zeros(number_workspace_sample_points, 2);
sample_index = 1;

for x_index = 1:numel(x_span)
    for y_index = 1:numel(y_span)
        xy_sample_points(sample_index, :) = ...
            [x_span(x_index) y_span(y_index)];
        sample_index = sample_index + 1;
    end
end

% L2 Span: 0 to 500 mm with 10 mm increments [units: m]
L2_span = 0.010:0.010:0.500; 

% L3 Span: 0 to 500 mm with 10 mm increments [units: m]
L3_span = 0.010:0.010:0.500;

L2_L3_check = ones(50,50);

T_0_corner = zeros(4, 4, 4);

T_0_corner(:, :, 1) = [1   0   0    x_lower;...
                       0   1   0    y_lower;...
                       0   0   1    0.155;...
                       0   0   0    1];
           
T_0_corner(:, :, 2) = [1   0   0    x_upper;...
                       0   1   0    y_lower;...
                       0   0   1    0.155;...
                       0   0   0    1];
           
T_0_corner(:, :, 3) = [1   0   0    x_upper;...
                       0   1   0    y_upper;...
                       0   0   1    0.155;...
                       0   0   0    1];
           
T_0_corner(:, :, 4) = [1   0   0    x_lower;...
                       0   1   0    y_upper;...
                       0   0   1    0.155;...
                       0   0   0    1];

number_L2_samples = numel(L2_span);
number_L3_samples = numel(L3_span);
 
if find_valid_link_pair == 1
    parfor L2_index = 1:number_L2_samples
        for L3_index = 1:number_L3_samples
            L2_j = L2_span(L2_index);
            L3_j = L3_span(L3_index); 
       
            % Feasible Link Flag 
            % (0: Infeasible Link Pair, 1: Feasible Link Pair) 
            feasible_links_boolean = 1;
       
            scara_j = mdl_scara_kuka_kr_6_r700z200(L1, L2_j, L3_j, Ltool);
       
            for workspace_corner = 1:4
                T_0_corner(:, :, workspace_corner);
                [q_i, infeasable_flag] = ...
                    ik_scara_kuka_kr_6_r700z200(scara_j, ...
                    T_0_corner(:, :, workspace_corner), ...
                    T_wrist_tool_value, 'deg', 'elbowOption','up');
                if infeasable_flag == 1
                    L2_L3_check(L2_index, L3_index) = 0;
                    feasible_links_boolean = 0;
                    break;
                end
            end
       
            if feasible_links_boolean == 0
                continue;
            end

        end
    end
else
    load('Valid_L2_L3_Pair.mat')
end

%%
goal_function = zeros(number_L2_samples, number_L3_samples);

tic
parfor L2_index = 1:number_L2_samples
    for L3_index = 1:number_L3_samples
        if L2_L3_check(L2_index, L3_index) == 1
            L2_j = L2_span(L2_index);
            L3_j = L3_span(L3_index);
            
            scara_j = mdl_scara_kuka_kr_6_r700z200(L1, L2_j, L3_j, Ltool);
            
            k = zeros(number_workspace_sample_points, 1);
            
            for sample_index_i = 1:number_workspace_sample_points
                xy_point = xy_sample_points(sample_index_i, :);
                
                T_0_i = [1   0   0    xy_point(1);...
                         0   1   0    xy_point(2);...
                         0   0   1    0.155;...
                         0   0   0    1];
                [q_i, infeasable_flag] = ...
                    ik_scara_kuka_kr_6_r700z200(scara_j, T_0_i, ...
                    T_wrist_tool_value, 'deg', 'elbowOption','up');
                
                if infeasable_flag == 1
                    disp('Help')
                end
                
                J_i = jacobian_scara(scara_j, q_i, 'deg');

                eigenvalue_i = eig(J_i * J_i');
                
                eigenvalue_i_bool = eigenvalue_i > tolerance;
                
                eigenvalue_i_reduced = eigenvalue_i(eigenvalue_i_bool);
                
                max_eigenvalue = max(eigenvalue_i_reduced);
                
                min_eigenvalue = min(eigenvalue_i_reduced);
                
                k(sample_index_i) = ...
                    sqrt(min_eigenvalue)/sqrt(max_eigenvalue);
                
            end
            
            sum_k = sum(k);
            min_k = min(k);
            
            goal_function(L2_index, L3_index) = ...
                (sum_k * min_k)/((L2_j)^3 + (L3_j)^3);
            
        end
    end
end
toc

%% Plotting the Goal Function as a function of L2/L3 Link Lengths
[L2_grid, L3_grid] = meshgrid(0.010:0.010:0.500);

figure(1)
imagesc(L2_span, L3_span, goal_function); 
axis equal
xlim([0.01 0.5])
ylim([0.01 0.5])
title('Goal Function vs $L_{2}$, $L_{3}$', ...
    'FontSize',14,'Interpreter','Latex')
xlabel('$L_{2}$ Link Length (m)', ...
    'FontSize',14,'Interpreter','Latex')
ylabel('$L_{3}$ Link Length (m)', ...
    'FontSize',14,'Interpreter','Latex')
colorbar

%% Question 4 (Part c)

goal_function_results = zeros(number_L2_samples * number_L3_samples, 3);

L2_L3_sample_index = 1;

tic
for L2_index = 1:number_L2_samples
    for L3_index = 1:number_L3_samples
        L2_j = L2_span(L2_index);
        L3_j = L3_span(L3_index);
        
        goal_function_results(L2_L3_sample_index, 1:2) = [L2_j L3_j];
        if L2_L3_check(L2_index, L3_index) == 1
            goal_function_results(L2_L3_sample_index, 3) = ...
                goal_function(L2_index, L3_index);
        end
        
        L2_L3_sample_index = L2_L3_sample_index + 1;
    end
end
toc

valid_goal_function_results_boolean = goal_function_results(:, 3) > 0;

valid_goal_function_results = ...
    goal_function_results(valid_goal_function_results_boolean, :);

max_goal_function_value = max(valid_goal_function_results(:, 3));

min_goal_function_value = min(valid_goal_function_results(:, 3));

max_goal_function_index = ...
    find(valid_goal_function_results(:, 3) == max_goal_function_value);

min_goal_function_index = ...
    find(valid_goal_function_results(:, 3) == min_goal_function_value);

best_L2_L3 = valid_goal_function_results(max_goal_function_index, 1:2);
worst_L2_L3 = valid_goal_function_results(min_goal_function_index, 1:2);

%% Best Combination for L2/L3 Link Lengths

% Best Combination L2, L3 Pair
L2_best = best_L2_L3(1);
L3_best = best_L2_L3(2);

scara_best = mdl_scara_kuka_kr_6_r700z200(L1, L2_best, L3_best, Ltool);

k_best = zeros(numel(y_span), numel(x_span));
q2_best = zeros(numel(y_span), numel(x_span));

tic
parfor y_index = 1:num_y_sample_points
    for x_index = 1:num_x_sample_points
        x_point = x_span(x_index);
        y_point = y_span(y_index);
        
        T_0_i = [1   0   0    x_point;...
                 0   1   0    y_point;...
                 0   0   1    0.155;...
                 0   0   0    1];
                
        [q_i, infeasable_flag] = ...
            ik_scara_kuka_kr_6_r700z200(scara_best, T_0_i, ...
            T_wrist_tool_value, 'deg', 'elbowOption', 'up');
        
        q2_best(y_index, x_index) = q_i(2);        
        if infeasable_flag == 1
            disp('Help')
        end
                
        J_i = jacobian_scara(scara_best, q_i, 'deg');
        
        eigenvalue_i = eig(J_i * J_i');
                
        eigenvalue_i_bool = eigenvalue_i > tolerance;
                
        eigenvalue_i_reduced = eigenvalue_i(eigenvalue_i_bool);
                
        max_eigenvalue = max(eigenvalue_i_reduced);
                
        min_eigenvalue = min(eigenvalue_i_reduced);
                
        k_best(y_index, x_index) = ...
            sqrt(min_eigenvalue)/sqrt(max_eigenvalue);
    end
end
toc

%% Plotting the Manipulabity of the Best Combination
figure(2)
imagesc(x_span, y_span, k_best); 
axis equal
hold on
plot([0.20 0.34], [-0.05 -0.05],'--r', 'LineWidth', 2)
plot([0.20 0.20], [-0.05  0.05],'--r', 'LineWidth', 2)
plot([0.20 0.34], [ 0.05  0.05],'--r', 'LineWidth', 2)
plot([0.34 0.34], [-0.05  0.05],'--r', 'LineWidth', 2)
xlim([0.195 0.345])
ylim([-0.055 0.055])
title('$\kappa_{i}$ vs Workspace (Best $L_{2}$, $L_{3}$ Combination)',...
    'FontSize',14,'Interpreter','Latex')
xlabel('$X$ Workspace Span (m)', ...
    'FontSize',14,'Interpreter','Latex')
ylabel('$Y$ Workspace Span (m)', ...
    'FontSize',14,'Interpreter','Latex')
colorbar
legend('Workspace Boundary')

%% Worst Combination for L2/L3 Link Lengths

% Worst Combination L2, L3 Pair
L2_worst = worst_L2_L3(1);
L3_worst = worst_L2_L3(2);

scara_worst = mdl_scara_kuka_kr_6_r700z200(L1, L2_worst, L3_worst, Ltool);

k_worst = zeros(numel(y_span), numel(x_span));
q2_worst = zeros(numel(y_span), numel(x_span));

parfor y_index = 1:num_y_sample_points
    for x_index = 1:num_x_sample_points
        x_point = x_span(x_index);
        y_point = y_span(y_index);
        
        T_0_i = [1   0   0    x_point;...
                 0   1   0    y_point;...
                 0   0   1    0.155;...
                 0   0   0    1];
                
        [q_i, infeasable_flag] = ...
            ik_scara_kuka_kr_6_r700z200(scara_worst, T_0_i, ...
            T_wrist_tool_value, 'deg', 'elbowOption', 'up');
        
        q2_worst(y_index, x_index) = q_i(2);        
        if infeasable_flag == 1
            disp('Help')
        end
                
        J_i = jacobian_scara(scara_worst, q_i, 'deg');
        
        eigenvalue_i = eig(J_i * J_i');
                
        eigenvalue_i_bool = eigenvalue_i > tolerance;
                
        eigenvalue_i_reduced = eigenvalue_i(eigenvalue_i_bool);
                
        max_eigenvalue = max(eigenvalue_i_reduced);
                
        min_eigenvalue = min(eigenvalue_i_reduced);
                
        k_worst(y_index, x_index) = ...
            sqrt(min_eigenvalue)/sqrt(max_eigenvalue);
    end
end
toc

%% Plotting the Manipulabity of the Worst Combination
figure(3)
imagesc(x_span, y_span, k_worst); 
axis equal
hold on
plot([0.20 0.34], [-0.05 -0.05],'--r', 'LineWidth', 2)
plot([0.20 0.20], [-0.05  0.05],'--r', 'LineWidth', 2)
plot([0.20 0.34], [ 0.05  0.05],'--r', 'LineWidth', 2)
plot([0.34 0.34], [-0.05  0.05],'--r', 'LineWidth', 2)
xlim([0.195 0.345])
ylim([-0.055 0.055])
title('$\kappa_{i}$ vs Workspace (Worst $L_{2}$, $L_{3}$ Combination)',...
    'FontSize',14,'Interpreter','Latex')
xlabel('$X$ Workspace Span (m)', ...
    'FontSize',14,'Interpreter','Latex')
ylabel('$Y$ Workspace Span (m)', ...
    'FontSize',14,'Interpreter','Latex')
colorbar
legend('Workspace Boundary')

%% Plotting the Theta_2 Values

q2_best_vec = reshape(q2_best.',1,[]);
q2_worst_vec = reshape(q2_worst.',1,[]);

figure(4)
plot(q2_best_vec, 'LineWidth', 2)
hold on
plot(q2_worst_vec, 'LineWidth', 2)
xlabel('Sample Point Index in the Workspace', ...
    'FontSize',14,'Interpreter','Latex')
ylabel('$\theta_{2} (\deg)$', ...
    'FontSize',14,'Interpreter','Latex')
legend('Best', 'Worst','Interpreter','Latex')

%% Testing Jacobian Values (Comparing Jacobian Computation Time)

x_point = x_span(9);
y_point = y_span(3);
        
T_0_i = [1   0   0    x_point;...
         0   1   0    y_point;...
         0   0   1    0.155;...
         0   0   0    1];
                
[q_i, infeasable_flag] = ...
    ik_scara_kuka_kr_6_r700z200(scara_best, T_0_i, ...
    T_wrist_tool_value, 'deg', 'elbowOption','up');

disp('Custom Jacobian')
tic
J_test_0 = jacobian_scara(scara_best, q_i, 'deg');
toc

% Explicit 
J_exp_sym = jacobian_explicit_sym(scara_best);

disp('Jacobian (Explicit)')
tic
J_test_1 = jacobian_explicit(scara_best, q_i, 'deg', 'numeric');
toc

disp('Jacobian Symbolic (Explicit)')
tic
J_test_2 = jacobian_syms2numeric(scara_best, q_i, 'deg', J_exp_sym);
toc

% Velocity Propagation 
J_vp_sym = jacobian_velocity_propagation_sym(scara_best);

disp('Jacobian (Velocity Propagation)')
tic
J_test_3 = ...
    jacobian_velocity_propagation(scara_best, q_i, 'deg', 'numeric');
toc

disp('Jacobian Symbolic (Velocity Propagation)')
tic
J_test_4 = jacobian_syms2numeric(scara_best, q_i, 'deg', J_vp_sym);
toc
% Force Propagation
J_fp_sym = jacobian_force_propagation_sym(scara_best);

disp('Jacobian (Force Propagation)')
tic
J_test_5 = jacobian_force_propagation(scara_best, q_i, 'deg', 'numeric');
toc

disp('Jacobian Symbolic (Force Propagation)')
tic
J_test_6 = jacobian_syms2numeric(scara_best, q_i, 'deg', J_fp_sym);
toc