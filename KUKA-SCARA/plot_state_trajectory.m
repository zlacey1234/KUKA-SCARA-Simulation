function plot_state_trajectory(state_of_tool_trajectory, time_sample, dt)
    %%
    
    state_dot_trajectory = ...
        gradient_trajectory(state_of_tool_trajectory, dt);
    
    state_dotdot_trajectory = ...
        gradient_trajectory(state_dot_trajectory, dt);
    
    number_of_dimensions = numel(state_of_tool_trajectory(1, :));
    
    figure
    for dimension_index = 1:number_of_dimensions
        subplot(3, number_of_dimensions, dimension_index)
        plot(time_sample, state_of_tool_trajectory(:, dimension_index))
        
        subplot(3, number_of_dimensions, ...
            number_of_dimensions + dimension_index)
        plot(time_sample, state_dot_trajectory(:, dimension_index))
        
        subplot(3, number_of_dimensions, ...
            2*number_of_dimensions + dimension_index)
        plot(time_sample, state_dotdot_trajectory(:, dimension_index))
    end
    
end
        