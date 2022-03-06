function J = jacobian_explicit(robot, joints, varargin)
%%
    
%%  
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
    
    if strcmp(joint_value_type, 'numeric')
        J = zeros(6, number_of_joints);
    elseif strcmp(joint_value_type, 'symbolic')
        J = sym(zeros(6, number_of_joints));
    else 
        disp('Error: Not a valid jointValueType');
    end
    
    joint_0_wrist_span = 1:1:number_of_joints;
    
    if strcmp(joint_value_type, 'numeric')
        T_0_wrist = robot.A(joint_0_wrist_span,... 
            joints);
    elseif strcmp(joint_value_type, 'symbolic')   
        T_0_wrist = simplify(robot.A(joint_0_wrist_span,... 
            joints));
    end
    
    T_0_wrist_matrix = T_0_wrist.T;
    
    P_0_wrist = T_0_wrist_matrix(1:3, 4);
    
    for joint_index = 1:number_of_joints
        
        joint_0_i_span = 1:1:joint_index;
        if strcmp(joint_value_type, 'numeric')
            T_0_i = robot.A(joint_0_i_span, joints);
        else
            T_0_i = simplify(robot.A(joint_0_i_span, joints));
        end
        
        T_0_i_matrix = T_0_i.T;
        
        P_0_i = T_0_i_matrix(1:3, 4);
        
        Z_0_i = T_0_i_matrix(1:3, 3);
        
        if L(joint_index).isrevolute
            Jv_0_i = cross(Z_0_i, (P_0_wrist - P_0_i));
            Jw_0_i = Z_0_i;
        else
            Jv_0_i = Z_0_i;
            Jw_0_i = zeros(3,1);
        end
        
        J(:, joint_index) = [Jv_0_i; Jw_0_i];
        
    end
    
    if strcmp(joint_value_type, 'symbolic')
        J = simplify(J);
    end
    
end