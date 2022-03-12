function [q_trajectory, infeasible_ik_boolean] = ...
    ik_puma560(robot, T_tool, angleType, varargin)
%% Function: ik_puma560
% Summary: The ik_puma560 function takes the inverse kinematics of the 
%          PUMA 560 Robotic Arm
%
% INPUTS: 
%   robot:     The robot object that was created using Peter Corke's
%              Robotics Toolbox in MATLAB (using the SerialLink Object
%              Class).   
%
%   T_tool:    A matrix containing the pose (position and orientation) of
%              the tool in the Homogeneous Transformation Representation. 
%
%   'angleType', {'deg', 'rad'}:         This allows the user to specify
%                                        what angle measurements type to
%                                        return the joint trajectory in. 
%
% OPTIONAL INPUTS: 
%   'linkSideOption', {'left', 'right'}  This is the option to specify if
%                                        the ik solution for the joint
%                                        (theta1) makes the arm link
%                                        position to the left or right
%                                        since the ik has two possible 
%                                        solutions for the joint. 
%
%   'elbowOption', {'up', 'down'}:       This is the option to specify if
%                                        the ik solution for the elbow
%                                        joint (theta3) is up or down 
%                                        since the ik has two possible 
%                                        solutions for the elbow joint. 
%   
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
    % Default Options
    defaultElbow = 'up';                   % theta3 joint
    expectedElbow = {'up', 'down'};        % Possible Elbow Options
    defaultLinkSide = 'left';              % theta1 joint
    expectedLinkSide = {'left', 'right'};  % Possible Link Side Options
    expectedAngleTypes = {'deg', 'rad'};
    
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'T_tool', @(x)(true));
    addRequired(p, 'angleType', ...
        @(x) any(validatestring(x, expectedAngleTypes)));
    addOptional(p, 'elbowOption', defaultElbow, ...
        @(x) any(validatestring(x, expectedElbow)));
    addOptional(p, 'linkSideOption', defaultLinkSide, ...
        @(x) any(validatestring(x, expectedLinkSide)));
    
    parse(p, robot, T_tool, angleType, varargin{:});
    
    elbow = validatestring(p.Results.elbowOption, expectedElbow);
    link_side = validatestring(p.Results.linkSideOption, ...
        expectedLinkSide);
    angle_type = validatestring(p.Results.angleType, expectedAngleTypes);
    
    tolerance = 1e-6;
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % Homogeneous Transformation from the Wrist Frame {n} to the Tool
    % Frame {T}. 
    T_n_tool = robot.tool;
    T_wrist_tool = T_n_tool.T;
    
    % Unpack the Homogenous Tool Point (or Trajectory)
    T_tool = SE3(T_tool);
    
    % Number of Trajectory Points
    number_of_trajectory_points = length(T_tool);
    
    % Initialize Joint Space Matrix (N x n) matrix where 'N' is the number 
    % of trajectory points and 'n' is the number of joints.
    q_trajectory = zeros(number_of_trajectory_points, number_of_joints);
    
    % Infeasible Inverse Kinematics Flag (0: Valid, 1: Invalid)
    infeasible_ik_boolean = 0;
    
    % For each trajectory point
    for trajectory_point_index = 1:number_of_trajectory_points
        
        % Possible Solutions 
        theta_solutions = zeros(8, number_of_joints);
        
        % Unpack the Homogeneous Transformation (4 x 4) matrix of the tool
        % w.r.t the Base Frame of the PUMA 560 Arm.
        T_tool_current = T_tool(trajectory_point_index);
        
        % Propagates from the Tool Frame {T} to the Wrist Frame
        % Convert back to a Matrix Form SE3 ---> Matrix in MATLAB
        T_wrist_current_matrix = T_tool_current.T * inv(T_wrist_tool);
        
        %% Analytical Solution to the Inverse Kinematics of the PUMA 560 
        % Unpack the Robot Object DH-Parameters
        a2 = L(3).a
        d3 = L(3).d
        a3 = L(4).a
        d4 = L(4).d
        
        % Upack Current Homogenous Tool Pose
        % Position of the tool (XYZ) w.r.t the Base Frame {B} of PUMA 560
        Px = T_wrist_current_matrix(1,4);
        Py = T_wrist_current_matrix(2,4);
        Pz = T_wrist_current_matrix(3,4);
        
        r11 = T_wrist_current_matrix(1,1);
        r21 = T_wrist_current_matrix(2,1);
        r31 = T_wrist_current_matrix(3,1);
        
        r12 = T_wrist_current_matrix(1,2);
        r22 = T_wrist_current_matrix(2,2);
        r32 = T_wrist_current_matrix(3,2);
        
        r13 = T_wrist_current_matrix(1,3);
        r23 = T_wrist_current_matrix(2,3);
        r33 = T_wrist_current_matrix(3,3);
        
        %% Solve for theta1
        rho = sqrt(Px^2 + Py^2);
        
        % Option 1 for theta1
        theta1_1 = atan2(Py, Px) - atan2(d3/rho, sqrt(1 - (d3^2/rho^2)));       
        
        % Option 2 for theta1
        theta1_2 = atan2(Py, Px) - atan2(d3/rho, -sqrt(1 - (d3^2/rho^2)));
        
        %% Solve for theta3
        K = (Px^2 + Py^2 + Pz^2 - a2^2 - a3^2 - d3^2 - d4^2)/(2 * a2);
        
        % Option 1 for theta3
        theta3_1 = atan2(a3, d4) - atan2(K, sqrt(a3^2 + d4^2 - K^2))
        
        % Option 2 for theta3
        theta3_2 = atan2(a3, d4) - atan2(K, -sqrt(a3^2 + d4^2 - K^2))
        
        %% Thus far, there are Four Possible Solutions (Solve for theta2):
        % Solution #1: function of (theta1_1, theta3_1)
        %
        % Solution #2: function of (theta1_2, theta3_1)
        %
        % Solution #3: function of (theta1_1, theta3_2)
        %
        % Solution #4: function of (theta1_2, theta3_2)
        
        % Solution #1
        [theta2_1, theta23_1] = solve_theta2(robot, ...
            [theta1_1 theta3_1], T_wrist_current_matrix);
        
        % Solution #2
        [theta2_2, theta23_2] = solve_theta2(robot, ...
            [theta1_2 theta3_1], T_wrist_current_matrix);
        
        % Solution #3
        [theta2_3, theta23_3] = solve_theta2(robot, ...
            [theta1_1 theta3_2], T_wrist_current_matrix);
        
        % Solution #4
        [theta2_4, theta23_4] = solve_theta2(robot, ...
            [theta1_2 theta3_2], T_wrist_current_matrix);
        
        %% Solve for theta4 
        % Solution #1: function of (theta1_1, theta2_1, theta3_1)
        %
        % Solution #2: function of (theta1_2, theta2_2, theta3_1)
        %
        % Solution #3: function of (theta1_1, theta2_3, theta3_2)
        %
        % Solution #4: function of (theta1_2, theta2_4, theta3_2)
        %
        % Solution #5: is Solution #1 Flipped (theta4_1 + pi)
        %
        % Solution #6: is Solution #2 Flipped (theta4_2 + pi)
        %
        % Solution #7: is Solution #3 Flipped (theta4_3 + pi)
        %
        % Solution #8: is Solution #4 Flipped (theta4_4 + pi)
        
        % [Solution #1, Solution #5]
        [theta4_1, theta4_5] = solve_theta4(robot, ...
            [theta1_1 theta2_1 theta3_1], ...
            T_wrist_current_matrix);
        
        % [Solution #2, Solution #6]
        [theta4_2, theta4_6] = solve_theta4(robot, ...
            [theta1_2 theta2_2 theta3_1], ...
            T_wrist_current_matrix);
        
        % [Solution #3, Solution #7]
        [theta4_3, theta4_7] = solve_theta4(robot, ...
            [theta1_1 theta2_3 theta3_2], ...
            T_wrist_current_matrix);
        
        % [Solution #4, Solution #8]
        [theta4_4, theta4_8] = solve_theta4(robot, ...
            [theta1_2 theta2_4 theta3_2], ...
            T_wrist_current_matrix);
        
        %% Solve for theta5 
        % Solution #1: 
        %   function of (theta1_1, theta2_1, theta3_1, theta4_1)
        %
        % Solution #2: 
        %   function of (theta1_2, theta2_2, theta3_1, theta4_2)
        %
        % Solution #3: 
        %   function of (theta1_1, theta2_3, theta3_2, theta4_3)
        %
        % Solution #4: 
        %   function of (theta1_2, theta2_4, theta3_2, theta4_4)
        %
        % Solution #5: is Solution #1 Flipped (-theta5_1)
        %
        % Solution #6: is Solution #2 Flipped (-theta5_2)
        %
        % Solution #7: is Solution #3 Flipped (-theta5_3)
        %
        % Solution #8: is Solution #4 Flipped (-theta5_4)
        
        % [Solution #1, Solution #5]
        [theta5_1, theta5_5] = solve_theta5(robot, ...
            [theta1_1 theta2_1 theta3_1 theta4_1], ...
            T_wrist_current_matrix);
        
        % [Solution #2, Solution #6]
        [theta5_2, theta5_6] = solve_theta5(robot, ...
            [theta1_2 theta2_2 theta3_1 theta4_2], ...
            T_wrist_current_matrix);
        
        % [Solution #3, Solution #7]
        [theta5_3, theta5_7] = solve_theta5(robot, ...
            [theta1_1 theta2_3 theta3_2 theta4_3], ...
            T_wrist_current_matrix);
        
        % [Solution #4, Solution #8]
        [theta5_4, theta5_8] = solve_theta5(robot, ...
            [theta1_2 theta2_4 theta3_2 theta4_4], ...
            T_wrist_current_matrix);
        
        %% Solve for theta5 
        % Solution #1: 
        %   function of (theta1_1, theta2_1, theta3_1, theta4_1, theta5_1)
        %
        % Solution #2: 
        %   function of (theta1_2, theta2_2, theta3_1, theta4_2, theta5_2)
        %
        % Solution #3: 
        %   function of (theta1_1, theta2_3, theta3_2, theta4_3, theta5_3)
        %
        % Solution #4: 
        %   function of (theta1_2, theta2_4, theta3_2, theta4_4, theta5_4)
        %
        % Solution #5: is Solution #1 Flipped (theta6_1 + pi)
        %
        % Solution #6: is Solution #2 Flipped (theta6_2 + pi)
        %
        % Solution #7: is Solution #3 Flipped (theta6_3 + pi)
        %
        % Solution #8: is Solution #4 Flipped (theta6_4 + pi)
        
        % [Solution #1, Solution #5]
        [theta6_1, theta6_5] = solve_theta6(robot, ...
            [theta1_1 theta2_1 theta3_1 theta4_1 theta5_1], ...
            T_wrist_current_matrix);
        
        % [Solution #2, Solution #6]
        [theta6_2, theta6_6] = solve_theta6(robot, ...
            [theta1_2 theta2_2 theta3_1 theta4_2 theta5_2], ...
            T_wrist_current_matrix);
        
        % [Solution #3, Solution #7]
        [theta6_3, theta6_7] = solve_theta6(robot, ...
            [theta1_1 theta2_3 theta3_2 theta4_3 theta5_3], ...
            T_wrist_current_matrix);
        
        % [Solution #4, Solution #8]
        [theta6_4, theta6_8] = solve_theta6(robot, ...
            [theta1_2 theta2_4 theta3_2 theta4_4 theta5_4], ...
            T_wrist_current_matrix);
        
%         % Solve for theta2
%         theta23 = atan2((-a3 - a2*cos(theta3))*Pz + ...
%             (cos(theta1)*Px + sin(theta1)*Py) * (a2*sin(theta3) - d4), ...
%             (a2*sin(theta3) - d4)*Pz - ...
%             (-a3 - a2*cos(theta3)) * (cos(theta1)*Px + sin(theta1)*Py));
%         
%         theta2 = theta23 - theta3
        
%         % Solve for theta4
%         theta4 = atan2(-r13*sin(theta1) + r23*cos(theta1), ...
%             -r13*cos(theta1)*cos(theta23) - ...
%             r23*sin(theta1)*cos(theta23) + sin(theta23)*r33)
%         
%         
%         %%
%         T_0_3 = robot.A([1 2 3], [theta1 theta2 theta3])
%         
%         inverse_homogeneous_matrix(T_0_3.T, 'numeric')
%         
%         T_3_6_temp = inverse_homogeneous_matrix(T_0_3.T, 'numeric') * ...
%             T_wrist_current_matrix
%         
%         if (abs(T_3_6_temp(1, 3)) < tolerance) & ...
%                 (abs(T_3_6_temp(3, 3)) < tolerance)
%             disp('Singularity')
%         else
%             theta4_test = atan2(T_3_6_temp(3, 3), -T_3_6_temp(1, 3))
%         end
        
        theta_solutions(1, :) = ...
            [theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1];
        
        theta_solutions(2, :) = ...
            [theta1_2 theta2_2 theta3_1 theta4_2 theta5_2 theta6_2];
        
        theta_solutions(3, :) = ...
            [theta1_1 theta2_3 theta3_2 theta4_3 theta5_3 theta6_3];
        
        theta_solutions(4, :) = ...
            [theta1_2 theta2_4 theta3_2 theta4_4 theta5_4 theta6_4];
        
        theta_solutions(5, :) = ...
            [theta1_1 theta2_1 theta3_1 theta4_5 theta5_5 theta6_5];
        
        theta_solutions(6, :) = ...
            [theta1_2 theta2_2 theta3_1 theta4_6 theta5_6 theta6_6];
        
        theta_solutions(7, :) = ...
            [theta1_1 theta2_3 theta3_2 theta4_7 theta5_7 theta6_7];
        
        theta_solutions(8, :) = ...
            [theta1_2 theta2_4 theta3_2 theta4_8 theta5_8 theta6_8];
        
        theta_solutions
        
        exceed_joint_limit_count = zeros(8, 1);
        
        for joint_index = 1:number_of_joints
            joint_limit = L(joint_index).qlim
            
            theta_joint = theta_solutions(:, joint_index)
            
            exceed_joint_limit_bool = ...
                abs(theta_joint) - joint_limit(2) > tolerance
            
            exceed_joint_limit_count = ...
                exceed_joint_limit_count + exceed_joint_limit_bool
        end
        
        reduced_valid_theta_solutions = ...
            theta_solutions(exceed_joint_limit_count == 0, :)
%         theta4_test = 
    end
    
    q_trajectory = 0
end

function [theta2, theta23] = solve_theta2(robot, theta_1_3, T_0_wrist)
%%
%%  
    % Unpack the theta1 and theta3 values
    theta1 = theta_1_3(1);
    theta3 = theta_1_3(2);
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Unpack the Robot Object DH-Parameters
    a2 = L(3).a;
    d3 = L(3).d;
    a3 = L(4).a;
    d4 = L(4).d;
    
    % Upack Current Homogenous Tool Pose
    % Position of the tool (XYZ) w.r.t the Base Frame {B} of PUMA
    Px = T_0_wrist(1,4);
    Py = T_0_wrist(2,4);
    Pz = T_0_wrist(3,4);
    
    theta23 = atan2((-a3 - a2*cos(theta3))*Pz + ...
        (cos(theta1)*Px + sin(theta1)*Py) * (a2*sin(theta3) - d4), ...
        (a2*sin(theta3) - d4)*Pz - ...
        (-a3 - a2*cos(theta3)) * (cos(theta1)*Px + sin(theta1)*Py));
    
    theta2 = theta23 - theta3;
end

function [theta4_1, theta4_2] = solve_theta4(robot, theta1_2_3, T_0_wrist)
%%

%%
    tolerance = 1e-6;
    
    T_0_3_ = robot.A([1 2 3], theta1_2_3)
    
    T_3_6_temp_ = inverse_homogeneous_matrix(T_0_3_.T, 'numeric') * ...
            T_0_wrist
        
    if (abs(T_3_6_temp_(1, 3)) < tolerance) & ...
            (abs(T_3_6_temp_(3, 3)) < tolerance)
        disp('Singularity')
    else
        theta4_1 = atan2(T_3_6_temp_(3, 3), -T_3_6_temp_(1, 3))
        
        theta4_2 = theta4_1 + pi
    end
end

function [theta5_1, theta5_2] = ...
    solve_theta5(robot, theta1_2_3_4, T_0_wrist)
%%

%%
    tolerance = 1e-6;
    
    T_0_4 = robot.A([1 2 3 4], theta1_2_3_4)
    
    T_4_6_temp_ = inverse_homogeneous_matrix(T_0_4.T, 'numeric') * ...
            T_0_wrist
        
    theta5_1 = atan2(-T_4_6_temp_(1, 3), T_4_6_temp_(3, 3))
    
    theta5_2 = -theta5_1
end

function [theta6_1, theta6_2] = ...
    solve_theta6(robot, theta1_2_3_4_5, T_0_wrist)
%%

%%
    tolerance = 1e-6;
    
    T_0_5 = robot.A([1 2 3 4 5], theta1_2_3_4_5)
    
    T_5_6_temp_ = inverse_homogeneous_matrix(T_0_5.T, 'numeric') * ...
            T_0_wrist
        
    theta6_1 = atan2(-T_5_6_temp_(3, 1), T_5_6_temp_(1, 1))
    
    theta6_2 = theta6_1 + pi
end