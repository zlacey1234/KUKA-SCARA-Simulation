function J = ...
    jacobian_force_propagation(robot, joints, angleType, varargin)
%% Function: Jacobian_Force_Propagation
% Summary: This is a function that iteratively solves for the Kinematic
%          Jacobian Matrix using the Force Propagation Method.
%
% Important Note:
%   It is recommended to avoid using this function for numeric 
%   calculations of the Kinematic Jacobian matrix. It may be useful to 
%   then create a custom jacobian function specific for the robot (i.e., 
%   create a method similar to jacobian_scara_rh3frh5515 example but for 
%   the robot being developed for). This custom jacobian will likely be 
%   more computationally efficient to integrate to trajectory generation 
%   or control methods that may require repeated calculations of the 
%   Jacobian Matrix (at various instances in time).
%
% INPUT: 
%   robot: @SerialLink Object created using the Robotics Toolbox by Peter
%          Corke.
%
%   joints: Joint Values 
%
% OUTPUT:
%   J: A (6 x N) matrix that represents the kinematic Jacobian of the
%      Serial Robot. N is the number of active joints in the kinematic
%      serial chain.
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
    % Default Options
    defaultJointValueType = 'symbolic';
    expectedJointValueType = {'symbolic', 'numeric'};
    expectedAngleTypes = {'deg', 'rad'};

    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'joints', @(x)(true));
    addRequired(p, 'angleType', ...
        @(x) any(validatestring(x, expectedAngleTypes)))
    addOptional(p, 'jointValueType', defaultJointValueType, ...
        @(x) any(validatestring(x, expectedJointValueType)));
    
    parse(p, robot, joints, angleType, varargin{:});
    
    joint_value_type = ...
        validatestring(p.Results.jointValueType, expectedJointValueType); 
    angle_type = validatestring(p.Results.angleType, expectedAngleTypes);

    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % Force/Torque applied by End-Effector 
    force_torque = sym('ft%d', [1 6], 'real');
    
    joint_ft = sym(zeros(number_of_joints, 1));

    if strcmp(joint_value_type, 'numeric')
        joints_numeric = joints;
        joints = sym('q%d', [1 number_of_joints], 'real');
    end
    
    f_i_i = sym(zeros(3, number_of_joints));
    n_i_i = sym(zeros(3, number_of_joints));
    
    joint_0_wrist_span = 1:1:number_of_joints;
    
    T_0_N = simplify(robot.A(joint_0_wrist_span,... 
        joints));
    
    T_0_N_matrix = T_0_N.T;
    
    P_0_N = T_0_N_matrix(1:3, 4);
    
    R_0_N = T_0_N_matrix(1:3, 1:3);
    
    % Initial Force applied by the End-Effector.
    f_i_i(:, number_of_joints) = ...
        [force_torque(1); force_torque(2); force_torque(3)];
    
    n_i_i(:, number_of_joints) = ...
        [force_torque(4); force_torque(5); force_torque(6)];
    
    for joint_index = (number_of_joints - 1):-1:1
        % iprev is equivalent to (joint_index) or (i)
        % icurr is equivalent to (joint_index + 1) or (i + 1)
        T_iprev_icurr = simplify(robot.A(joint_index + 1, joints));
        
        T_iprev_icurr_matrix = T_iprev_icurr.T;
        
        R_iprev_icurr = T_iprev_icurr_matrix(1:3, 1:3);
        
        P_iprev_icurr = T_iprev_icurr_matrix(1:3, 4);
        
        f_i_i(:, joint_index) = ...
            R_iprev_icurr * f_i_i(:, joint_index  + 1);
        
        n_i_i(:, joint_index) = ...
            R_iprev_icurr * n_i_i(:, joint_index  + 1) + ...
            cross(P_iprev_icurr, f_i_i(:, joint_index));
    end
    
    for joint_index = number_of_joints:-1:1
        if L(joint_index).isrevolute
            joint_ft(joint_index) = ...
                transpose(n_i_i(:, joint_index)) * [0; 0; 1];
        else
            joint_ft(joint_index) = ...
                transpose(f_i_i(:, joint_index)) * [0; 0; 1];
        end 
    end
    
    joint_ft = simplify(joint_ft);
    f_i_i = simplify(f_i_i);
    n_i_i = simplify(n_i_i);
    
    J_N_N_Transpose = equationsToMatrix(joint_ft, force_torque);
    
    J_N_N = transpose(J_N_N_Transpose);
    
    jacobian_rotation_0_N = [R_0_N zeros(3,3); zeros(3,3) R_0_N];
    
    J_0_N = simplify(jacobian_rotation_0_N * J_N_N);
    J = J_0_N;

    if strcmp(joint_value_type, 'numeric')
        for joint_index = 1:number_of_joints
            if L(joint_index).isrevolute
                if strcmp(angle_type, 'deg')
                    joints_numeric(joint_index) = ...
                        deg2rad(joints_numeric(joint_index));
                end
            end
            joint_string = ['q' num2str(joint_index)];
            J = subs(J, joint_string, joints_numeric(joint_index));
        end

        J = double(J);
    end
end