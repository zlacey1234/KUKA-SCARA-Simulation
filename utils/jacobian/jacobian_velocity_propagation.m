function J = jacobian_velocity_propagation(robot, joints, varargin)
%%

%%
    % Default Options
    defaultJointValueType = 'numeric';
    expectedJointValueType = {'symbolic', 'numeric'};
    
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'joints', @(x)(true));
    addOptional(p, 'jointValueType', defaultJointValueType, ...
        @(x) any(validatestring(x, expectedJointValueType)));
    
    parse(p, robot, joints, varargin{:});
    
    joint_value_type = ...
        validatestring(p.Results.jointValueType, expectedJointValueType); 
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % joint-dot
    dot_joint = sym('q%ddot', [1 number_of_joints], 'real');
    
    if strcmp(joint_value_type, 'numeric')
        w_i_i = zeros(3, number_of_joints + 1);
        v_i_i = zeros(3, number_of_joints + 1);
    elseif strcmp(joint_value_type, 'symbolic')
        w_i_i = sym(zeros(3, number_of_joints + 1));
        v_i_i = sym(zeros(3, number_of_joints + 1));
    else 
        disp('Error: Not a valid jointValueType');
    end
    
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
            disp('Revolute')
            w_i_i(:,joint_index + 1) = ...
                simplify(R_icurr_iprev * w_i_i(:, joint_index) + ...
                [0; 0; dot_joint(joint_index)]);
            
            v_i_i(:,joint_index + 1) = ...
                simplify(collect(R_icurr_iprev * ...
                (cross(w_i_i(:, joint_index), P_iprev_icurr) + ...
                v_i_i(:,joint_index)), dot_joint));
        else
            disp('Prismatic')
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
    
%     w_i_i
%     v_i_i
    
    w_N_N = simplify(w_i_i(:,number_of_joints + 1));
    v_N_N = simplify(v_i_i(:,number_of_joints + 1));
    
    [Jv_N_N] = equationsToMatrix(v_N_N, dot_joint);
    [Jw_N_N] = equationsToMatrix(w_N_N, dot_joint);
    
    J_N_N = [Jv_N_N; Jw_N_N]
    
    jacobian_rotation_0_N = [R_0_N zeros(3,3); zeros(3,3) R_0_N]
    
    J_0_N = simplify(jacobian_rotation_0_N * J_N_N);
    
    J = J_0_N;
end