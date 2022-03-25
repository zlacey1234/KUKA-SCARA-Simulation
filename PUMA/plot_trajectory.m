function plot_trajectory(robot, time_span, qrt, qrt_reference, ...
    phase_timestamps, angleType)
%%
    
%%
    % Default Options
    expectedAngleTypes = {'deg', 'rad'};
    
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'time_span', @(x)(true));
    addRequired(p, 'qrt', @(x)(true));
    addRequired(p, 'qrt_reference', @(x)(true));
    addRequired(p, 'phases_timestamps', @(x)(true));
    addRequired(p, 'angleType', ...
        @(x) any(validatestring(x, expectedAngleTypes)));
    
    parse(p, robot, time_span, qrt, qrt_reference, ...
        phase_timestamps, angleType)
    
    angle_type = validatestring(p.Results.angleType, expectedAngleTypes);
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    %% Calculations
    
    % Number of steps in the Trajectory
    num_steps = numel(time_span);
    
    % Number of Phases 
    num_phases = numel(phase_timestamps)
    
    % Phase Indices 
    phase_indices = zeros(num_phases + 1, 2);
    
    phase_indices(1, 2) = 1;
    
    start_phase_time = 0;
    for phase_index = 1:num_phases
        end_phase_time = phase_timestamps(phase_index);
        
        start_phase_time_index = find(time_span == start_phase_time);
        end_phase_time_index = find(time_span == end_phase_time);
        
        phase_indices(phase_index + 1, :) = ...
            [end_phase_time  end_phase_time_index];
        
        % Update start_phase 
        start_phase_time = phase_timestamps(phase_index);
    end
    
    % Calculating the Position XYZ and Orientation (3-2-1 Euler)
    XYZ_position = zeros(num_steps, 3);
    EulerAngle_ZYX = zeros(num_steps, 3);
    
    for step_index = 1:num_steps
        qrt_current = qrt(step_index, :);
        
        % Forward Kinematics to obtain the Homogeneous Transformation 
        % Matrix to the wrist. This is w.r.t the Base Frame {0}.
        T_0_6_current = robot.A([1 2 3 4 5 6], qrt_current);
        
        T_6_tool = robot.tool;
        
        T_6_tool_matrix = T_6_tool.T;
        
        T_0_6_current_matrix = T_0_6_current.T;
        
        T_0_tool_current = T_0_6_current_matrix * T_6_tool_matrix;
        
        [R_0_tool, P_0_tool] = tr2rt(T_0_tool_current);
        
        EulerAngle_current = rotm2eul(R_0_tool, 'ZYX');
        
        XYZ_position(step_index, :) = P_0_tool;
        
        EulerAngle_ZYX(step_index, :) = EulerAngle_current;
            
    end
    
    %% Position (X, Y, Z) 3D Trajectory Plot
    figure(10)
    
    for phase_index = 1:num_phases
        start_phase_index = phase_indices(phase_index, 2);
        end_phase_index = phase_indices(phase_index + 1, 2);
        
        plot3(XYZ_position(start_phase_index:end_phase_index, 1), ...
              XYZ_position(start_phase_index:end_phase_index, 2), ...
              XYZ_position(start_phase_index:end_phase_index, 3), ...
              'LineWidth', 2);
          
        hold on
        grid on
        axis equal
    end
    title('Position XYZ 3D Plot of the Trajectory')
    xlabel('X Position (meters)')
    ylabel('Y Position (meters)')
    zlabel('Z Position (meters)')
    legend('Phase 1', 'Phase 2', 'Phase 3')
    
    %% Euler Angles 
    
    figure(11)
    for phase_index = 1:num_phases
        start_phase_index = phase_indices(phase_index, 2);
        end_phase_index = phase_indices(phase_index + 1, 2);
        
        subplot(3, 1, 1)
        plot(time_span(start_phase_index:end_phase_index), ...
            rad2deg(EulerAngle_ZYX(start_phase_index:end_phase_index,...
            1)), ...
            'LineWidth', 2);
        hold on
        grid on
        legend('Phase 1', 'Phase 2', 'Phase 3')
        title('ZYX Euler Angles of the End-Effector w.r.t the Base Frame {0}')
        
        subplot(3, 1, 2)
        plot(time_span(start_phase_index:end_phase_index), ...
            rad2deg(EulerAngle_ZYX(start_phase_index:end_phase_index,... 
            2)), ...
            'LineWidth', 2);
        hold on
        grid on
        
        subplot(3, 1, 3)
        plot(time_span(start_phase_index:end_phase_index), ...
            rad2deg(EulerAngle_ZYX(start_phase_index:end_phase_index,... 
            3)), ...
            'LineWidth', 2);
        hold on
        grid on
        xlabel('Time t (seconds)')
    end
    
    %% XYZ Position 
    
    figure(12)
    for phase_index = 1:num_phases
        start_phase_index = phase_indices(phase_index, 2);
        end_phase_index = phase_indices(phase_index + 1, 2);
        
        subplot(3, 1, 1)
        plot(time_span(start_phase_index:end_phase_index), ...
            XYZ_position(start_phase_index:end_phase_index, 1), ...
            'LineWidth', 2);
        hold on
        grid on
        ylabel('X Position (meters)')
        legend('Phase 1', 'Phase 2', 'Phase 3')
        title('XYZ Position of the End-Effector w.r.t the Base Frame {0}')
        
        subplot(3, 1, 2)
        plot(time_span(start_phase_index:end_phase_index), ...
            XYZ_position(start_phase_index:end_phase_index, 2), ...
            'LineWidth', 2);
        hold on
        grid on
        ylabel('Y Position (meters)')
        
        subplot(3, 1, 3)
        plot(time_span(start_phase_index:end_phase_index), ...
            XYZ_position(start_phase_index:end_phase_index, 3), ...
            'LineWidth', 2);
        hold on
        grid on
        xlabel('Time t (seconds)')
        ylabel('Z Position (meters)')
    end
end