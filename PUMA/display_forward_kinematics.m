function display_forward_kinematics(robot, q_symbolic)
%% Function: Display_Forward_Kinematics

%%
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints (n)
    number_of_joints = robot.n();
    
    % Homogeneous Transformation Matrices for each Joint/Link Connection
    for joint_index = 1:number_of_joints
        % String to Display the Transformation
        transformation_T_string = ...
            ['T_' num2str(joint_index - 1) '_' num2str(joint_index)];
        
        T_current = robot.A(joint_index, q_symbolic);
        
        % Displays the Homogeneous Transformation Matrices
        disp(transformation_T_string)
        T_current
        
    end
    
    % Homogeneous Transformation from the Wrist Frame {n} to the Tool
    % Frame {T}. 
    T_n_tool = robot.tool;
    T_n_tool_matrix = simplify(T_n_tool.T);
    
    disp(['T_' num2str(joint_index) '_tool'])
    T_n_tool_matrix
    
    joint_span = 1:number_of_joints;
    
    % Homogeneous Transformation from the Base Frame {0} to the Wrist
    % Frame {n}.
    T_0_n = robot.A(joint_span, q_symbolic);
    T_0_n_matrix = simplify(T_0_n.T);
    
    disp(['T_0_' num2str(joint_index)])
    T_0_n_matrix
    
    % Homogeneous Transformation from the Base Frame {0} to the Tool
    % Frame {T}.
    T_0_tool_matrix = T_0_n_matrix * T_n_tool_matrix;
    
    disp('T_0_tool')
    T_0_tool_matrix
    
    disp(['inv(T_' num2str(joint_index) '_tool)'])
    T_tool_n = inverse_homogeneous_matrix(T_n_tool_matrix, 'symbolic')
end