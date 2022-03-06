function J = ...
    jacobian_velocity_propagation_sym(robot)
%% Function: Jacobian_Velocity_Propagation
% Summary: This is a function that iteratively solves for the Kinematic
%          Jacobian Matrix using the Velocity Propagation Method.
%
% Important Note:
%   This function should only really be used to obtain the symbolic
%   representation of the Jacobian Matrix. It may be useful to then create
%   a custom jacobian function specific for the robot (i.e., create a
%   method similar to jacobian_scara_rh3frh5515 example but for the robot
%   being developed for). This custom jacobian will likely be more
%   computationally efficient to integrate to trajectory generation or
%   control methods that may require repeated calculations of the Jacobian
%   Matrix (at various instances in time). 
%
% INPUTS: 
%   robot: @SerialLink Object created using the Robotics Toolbox by Peter
%          Corke.
%
% OUTPUTS:
%   J: A (6 x N) matrix that represents the kinematic Jacobian of the
%      Serial Robot. N is the number of active joints in the kinematic
%      serial chain. This returns the symbolic version of the kinematic
%      Jacobian matrix.
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com   
%%
    % Parser
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    
    parse(p, robot); 
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % joint-dot
    dot_joint = sym('q%ddot', [1 number_of_joints], 'real');
    
   
    joints = sym('q%d', [1 number_of_joints], 'real');

    w_i_i = sym(zeros(3, number_of_joints + 1));
    v_i_i = sym(zeros(3, number_of_joints + 1));
    
    joint_0_wrist_span = 1:1:number_of_joints;
    
    T_0_N = simplify(robot.A(joint_0_wrist_span,... 
        joints));
    
    T_0_N_matrix = T_0_N.T;
    
    P_0_N = T_0_N_matrix(1:3, 4);
    
    R_0_N = T_0_N_matrix(1:3, 1:3);
    
    for joint_index = 1:number_of_joints
        % iprev is equivalent to (joint_index) or (i)
        % icurr is equivalent to (joint_index + 1) or (i + 1)
        T_iprev_icurr = simplify(robot.A(joint_index, joints));
        
        T_iprev_icurr_matrix = T_iprev_icurr.T;
        
        R_iprev_icurr = T_iprev_icurr_matrix(1:3, 1:3);
        
        R_icurr_iprev = transpose(R_iprev_icurr);
        
        P_iprev_icurr = T_iprev_icurr_matrix(1:3, 4);
        
        if L(joint_index).isrevolute
%             disp('Revolute')
            w_i_i(:,joint_index + 1) = ...
                simplify(R_icurr_iprev * w_i_i(:, joint_index) + ...
                [0; 0; dot_joint(joint_index)]);
            
            v_i_i(:,joint_index + 1) = ...
                simplify(collect(R_icurr_iprev * ...
                (cross(w_i_i(:, joint_index), P_iprev_icurr) + ...
                v_i_i(:,joint_index)), dot_joint));
        else
%             disp('Prismatic')
            w_i_i(:,joint_index + 1) = ...
                simplify(R_icurr_iprev * w_i_i(:, joint_index));
            
            v_i_i(:,joint_index + 1) = ...
                collect(R_icurr_iprev * ...
                (cross(w_i_i(:, joint_index), P_iprev_icurr) + ...
                v_i_i(:,joint_index) + ...
                [0; 0; dot_joint(joint_index)]), ...
                dot_joint);
        end
    end
    
    w_N_N = simplify(w_i_i(:,number_of_joints + 1));
    v_N_N = simplify(v_i_i(:,number_of_joints + 1));
    
    [Jv_N_N] = equationsToMatrix(v_N_N, dot_joint);
    [Jw_N_N] = equationsToMatrix(w_N_N, dot_joint);
    
    J_N_N = [Jv_N_N; Jw_N_N];
    
    jacobian_rotation_0_N = [R_0_N zeros(3,3); zeros(3,3) R_0_N];
    
    J_0_N = simplify(jacobian_rotation_0_N * J_N_N);
    J = J_0_N;
end