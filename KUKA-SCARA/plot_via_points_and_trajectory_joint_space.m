function state_of_tool_trajectory = ...
    plot_via_points_and_trajectory_joint_space(robot, ...
    T_S_viapoints_matrix, T_Base_Station, q_trajectory, T_wrist_tool)
%%

    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % Number of Via Points
    number_of_via_points = numel(T_S_viapoints_matrix(1, 1, :));
    
    % Number of samples in the trajectory
    number_of_samples = numel(q_trajectory(:,1));
    
    for via_point_index = 1:number_of_via_points
        % Current Via Point (Homogeneous) w.r.t the Base Frame {0}
        T_S_current_via_point = ...
            T_S_viapoints_matrix(:, :, via_point_index);
        
        % Current Via Point (Homogeneous) w.r.t the Stationary Frame {S}
        T_0_current_via_point = T_Base_Station*T_S_current_via_point;
        
        position_of_via_point = T_0_current_via_point(1:3, 4);
        
        plot3(position_of_via_point(1), ...
            position_of_via_point(2), ...
            position_of_via_point(3),'y.')
    end
    
    position_of_tool_trajectory = zeros(number_of_samples, 3);
    
    state_of_tool_trajectory = zeros(number_of_samples, 6);
    
    joint_0_wrist_span = 1:1:number_of_joints;
    
    for sample_index = 1:number_of_samples
        q_current = q_trajectory(sample_index, :);
        
        T_0_Wrist = robot.A(joint_0_wrist_span, q_current);
        
        T_0_Tool_matrix = T_0_Wrist.T * T_wrist_tool;
        
        T_Station_Base = inverse_homogeneous_matrix(T_Base_Station);
        
        T_S_Tool_matrix = T_Station_Base*T_0_Tool_matrix
        
        state_of_tool_trajectory(sample_index, :) = ...
            transformation2equivalent_axis_angle(T_S_Tool_matrix);
        
        if state_of_tool_trajectory(sample_index,6) < -3.14
            T_S_Tool_matrix
        end
        
        position_of_tool_current = T_0_Tool_matrix(1:3, 4);
        
        position_of_tool_trajectory(sample_index, :) = ...
            position_of_tool_current;
    end
    
    state_of_tool_trajectory;
    
    
    plot3(position_of_tool_trajectory(:,1), ...
        position_of_tool_trajectory(:,2), ...
        position_of_tool_trajectory(:,3), 'c--');
    
end