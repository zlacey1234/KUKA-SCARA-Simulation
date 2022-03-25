function ...
    [qrt_final, time_span] = ...
    generate_trajectory_linear_parabolic(robot, T_S_viapoints_matrix, ...
    T_Base_Station, desired_time_intervals, desired_accelerations, ...
    time_resolution, T_wrist_tool, qrt_init) 
%%
%
% INPUTS:
%   robot:
% 
%   T_S_viapoints_matrix: A (4 x 4 x N) matrix where N is the number of 
%                         via points and the matrix contains the via 
%                         points as a set of Homogeneous Transformation 
%                         matrices. 
    
    generate_trajectory_bool = 1;

    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = length(L);
    
    % Number of Via Points
    number_of_via_points = numel(T_S_viapoints_matrix(1, 1, :));
    
    % Initialize the Matrix to store the Via Point (Joint Space
    % Representation.
    q_viapoints = zeros(number_of_via_points, number_of_joints);
    
    q_viapoints(1, :) = qrt_init;
    
    % For each Via Point
    for via_point_index = 2:number_of_via_points
       % Current Via Point (Homogeneous) w.r.t the Stationary Frame {S}
       T_S_current_via_point = T_S_viapoints_matrix(:, :, via_point_index);
       
       % Current Via Point (Homogeneous) w.r.t the Base Frame {0}
       T_0_current_via_point = T_Base_Station*T_S_current_via_point;
       
       % Obtain the Joint Space Representation of the Via Point by
       % calculating the Inverse Kinematics (necessary Joint Angles of the
       % SCARA Robot to get the tool to the defined Via Point poses).
       q_viapoints(via_point_index, :) = ...
           ik_puma560(robot, T_0_current_via_point, ...
           'deg', q_viapoints(via_point_index - 1, :));
    end
    
    % Desired total time of the Trajectory
    desired_trajectory_time = sum(desired_time_intervals);
    
    % Storing the values of the acceleration at the Via Points 
    acceleration_at_via = zeros(size(q_viapoints));
    
    % Time span of the parabolic blend segment at Via Point J (t_J) 
    blend_time_at_via = zeros(size(q_viapoints));
    
    % Storing the values of the velocity between the Via Points
    velocity_at_via = zeros(size(q_viapoints));
    
    % Time span of the linear segment after Via Point J (t_JK) 
    linear_time_after_via = zeros(size(q_viapoints));
    
    time_span = 0:time_resolution:desired_trajectory_time;
    
    qrt = zeros(numel(time_span), number_of_joints);
    qdotrt = zeros(numel(time_span), number_of_joints);
    qdotdotrt = zeros(numel(time_span), number_of_joints);
    
    % Keeps track of the absolute time of each instance that the segment
    % switches (i.e., switching from Linear to Parabolic). It is absolute
    % since it is the time w.r.t the beginning of the Trajectory.
    absolute_time_segment = zeros(1, number_of_joints);
    
    segments_start_time_index = zeros(1, number_of_joints);
    
    % For each Via Point until the Via Point before the last Via Point.
    for via_point_index = 1:number_of_via_points
        
        % For each Joint
        for joint_index = 1:number_of_joints
            
            % Calculating the 
            
            if via_point_index == 1 % First Segment
                [acceleration_1, blend_time_1, velocity_12] = ...
                    parabolic_blend_first_segment(...
                    q_viapoints(via_point_index, joint_index), ...
                    q_viapoints(via_point_index + 1, joint_index), ...
                    desired_time_intervals(via_point_index), ...
                    desired_accelerations(joint_index));
                
                acceleration_at_via(via_point_index, joint_index) = ...
                    acceleration_1;
                
                blend_time_at_via(via_point_index, joint_index) = ...
                    blend_time_1;
                
                velocity_at_via(via_point_index, joint_index) = ...
                    velocity_12;
                
            elseif via_point_index == number_of_via_points 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Last Segment 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                [acceleration_n, blend_time_n, velocity_to_n] = ...
                    parabolic_blend_last_segment(...
                    q_viapoints(via_point_index - 1, joint_index), ...
                    q_viapoints(via_point_index, joint_index), ...
                    desired_time_intervals(via_point_index - 1), ...
                    desired_accelerations(joint_index));
%                 
%                 [acceleration_k, blend_time_k, velocity_kl] = ...
%                     parabolic_blend_mid_points(...
%                     q_viapoints(via_point_index - 1, joint_index), ...
%                     q_viapoints(via_point_index, joint_index), ...
%                     q_viapoints(via_point_index + 1, joint_index), ...
%                     desired_time_intervals(via_point_index - 1), ...
%                     desired_time_intervals(via_point_index), ...
%                     desired_accelerations(joint_index), ...
%                     via_point_index, number_of_via_points);
                    
                acceleration_at_via(via_point_index, joint_index) = ...
                    acceleration_n;
                
%                 acceleration_at_via(via_point_index, joint_index) = ...
%                     acceleration_k;
                
                blend_time_at_via(via_point_index, joint_index) = ...
                    blend_time_n;
                
            else
                [acceleration_k, blend_time_k, velocity_kl] = ...
                    parabolic_blend_mid_points(...
                    q_viapoints(via_point_index - 1, joint_index), ...
                    q_viapoints(via_point_index, joint_index), ...
                    q_viapoints(via_point_index + 1, joint_index), ...
                    desired_time_intervals(via_point_index - 1), ...
                    desired_time_intervals(via_point_index), ...
                    desired_accelerations(joint_index), ...
                    via_point_index, number_of_via_points);
                
                acceleration_at_via(via_point_index, joint_index) = ...
                    acceleration_k;
                
                blend_time_at_via(via_point_index, joint_index) = ...
                    blend_time_k;
                
                velocity_at_via(via_point_index, joint_index) = ...
                    velocity_kl;
                    
            end
            
            if via_point_index == 1 % First Parabolic Segment
                t_1 = blend_time_at_via(via_point_index, joint_index);
                t_2 = blend_time_at_via(via_point_index + 1, joint_index);
                
                theta_0 = q_viapoints(via_point_index, joint_index);
                t_d12 = desired_time_intervals(via_point_index);
                v_12 = velocity_at_via(via_point_index, joint_index);
                a_1 = acceleration_at_via(via_point_index, joint_index);
                
                % Parabolic Segment for Current Segment
                time_duration_span_parabolic = 0:time_resolution:t_1; 
                
                number_of_samples_in_segment_parabolic_1 = ...
                    numel(time_duration_span_parabolic);
                
                % Time span of the parabolic segment relative to the 
                % initial via point (Via Point 0). Time is measured from
                % Via Point 0.
                relative_time_span_parabolic_1 = ...
                    time_duration_span_parabolic;
                
                absolute_time_span_parabolic_1 = ...
                    time_duration_span_parabolic + ...
                    absolute_time_segment(joint_index);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                % 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                joint_rt_parabolic_1 = theta_0 + ...
                    0.5*(v_12 / t_1)*relative_time_span_parabolic_1.^2;
                
                joint_velocity_rt_parabolic_1 = ...
                    a_1 * relative_time_span_parabolic_1;
                
                joint_acceleration_rt_parabolic_1 = ...
                    a_1 * ones(size(absolute_time_span_parabolic_1));
                
                qrt(1:number_of_samples_in_segment_parabolic_1, ...
                    joint_index) = joint_rt_parabolic_1;
                
                qdotrt(1:number_of_samples_in_segment_parabolic_1, ...
                    joint_index) = joint_velocity_rt_parabolic_1;
                
                qdotdotrt(1:number_of_samples_in_segment_parabolic_1, ...
                    joint_index) = joint_acceleration_rt_parabolic_1;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                % Plotting Information
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                figure(joint_index)
                
                title_string = ...
                    ['Information for Joint ' num2str(joint_index)];
                
                subplot(3, 1, 1)
                plot(absolute_time_span_parabolic_1, ...
                    joint_rt_parabolic_1, 'LineWidth', 2);
                hold on
                grid on
                title(title_string)
                if L(joint_index).isrevolute 
                    ylabel('Joint Value (degrees)')
                else 
                    ylabel('Joint Value (meters)')
                end
                
                subplot(3, 1, 2)
                plot(absolute_time_span_parabolic_1, ...
                    joint_velocity_rt_parabolic_1, 'LineWidth', 2);
                hold on
                grid on
                if L(joint_index).isrevolute 
                    ylabel('Joint Velocity (degrees/second)')
                else 
                    ylabel('Joint Velocity (meters/second)')
                end
                
                subplot(3, 1, 3)
                plot(absolute_time_span_parabolic_1, ...
                    joint_acceleration_rt_parabolic_1, 'LineWidth', 3);
                hold on
                grid on
                xlabel('Time (seconds)')
                if L(joint_index).isrevolute 
                    ylabel('Joint Acceleration (degrees/second^2)')
                else 
                    ylabel('Joint Acceleration (meters/second^2)')
                end
                absolute_time_segment(joint_index) = t_1;
                
                segments_start_time_index(joint_index) = ...
                    number_of_samples_in_segment_parabolic_1;
                
            else
                t_J = blend_time_at_via(via_point_index - 1, joint_index);
                t_K = blend_time_at_via(via_point_index, joint_index);
                
                theta_J = q_viapoints(via_point_index - 1, joint_index);
                t_dJK = desired_time_intervals(via_point_index - 1);
                v_JK = velocity_at_via(via_point_index - 1, joint_index);
                a_K = acceleration_at_via(via_point_index, ... 
                    joint_index);
                
                if (via_point_index - 1) == 1
                    linear_time_after_via(via_point_index - 1, ... 
                        joint_index) = t_dJK - t_J - 0.5 * t_K;
                elseif (via_point_index) == number_of_via_points
                    linear_time_after_via(via_point_index - 1, ... 
                        joint_index) = t_dJK - t_K - 0.5 * t_J;
                else
                    linear_time_after_via(via_point_index - 1, ... 
                        joint_index) = t_dJK - 0.5 * t_J - 0.5 * t_K;
                end
                
                t_JK = linear_time_after_via(via_point_index - 1, ...
                    joint_index);
                
                % Linear Segment for Previous Segment
                time_duration_span_linear = 0:time_resolution:t_JK; 
                
                number_of_samples_in_segment_linear_JK = ...
                    numel(time_duration_span_linear);
                
                % Time span of the linear segment relative to the 
                % previous via point (Via Point J [or via_point_index-1]). 
                % Time is measured from Via Point J.
                relative_time_span_linear_JK = ...
                    time_duration_span_linear + (t_J/2);
                
                absolute_time_span_linear_JK = ...
                    time_duration_span_linear + ...
                    absolute_time_segment(joint_index);
                
                qrt_linear = theta_J + v_JK * relative_time_span_linear_JK;
                
                joint_velocity_rt_linear_JK = ...
                    v_JK * ones(size(absolute_time_span_linear_JK)); 
                
                joint_acceleration_rt_linear_JK = ...
                    zeros(size(absolute_time_span_linear_JK));
                
                qrt(segments_start_time_index(joint_index):...
                    (segments_start_time_index(joint_index) + ...
                    number_of_samples_in_segment_linear_JK) - 1, ...
                    joint_index) = qrt_linear;
                
                qdotrt(segments_start_time_index(joint_index):...
                    (segments_start_time_index(joint_index) + ...
                    number_of_samples_in_segment_linear_JK) - 1, ...
                    joint_index) = joint_velocity_rt_linear_JK;
                
                qdotdotrt(segments_start_time_index(joint_index):...
                    (segments_start_time_index(joint_index) + ...
                    number_of_samples_in_segment_linear_JK) - 1, ...
                    joint_index) = joint_acceleration_rt_linear_JK;
                
                absolute_time_segment(joint_index) = ...
                    absolute_time_segment(joint_index) + t_JK;
                
                segments_start_time_index(joint_index) = ...
                    segments_start_time_index(joint_index) + ...
                    number_of_samples_in_segment_linear_JK;
                
                % Parabolic Segment for Current Segment
                time_duration_span_parabolic = 0:time_resolution:t_K; 
                
                number_of_samples_in_segment_parabolic_K = ...
                    numel(time_duration_span_parabolic);
                
                % Time span of the parabolic segment relative to the 
                % previous via point (Via Point J [or via_point_index-1]). 
                % Time is measured from Via Point J. 
                relative_time_span_parabolic_K = ...
                    time_duration_span_parabolic + (t_J/2 + t_JK);
                
                absolute_time_span_parabolic_K = ...
                    time_duration_span_parabolic + ...
                    absolute_time_segment(joint_index);
                
                t_in_b = relative_time_span_parabolic_K - (t_J/2 + t_JK);
                
                qrt_parabolic = theta_J + ...
                    v_JK * (t_J/2 + t_JK) + v_JK *(t_in_b) + ...
                    0.5 * (a_K) .* (t_in_b) .^ 2;
                
                joint_velocity_rt_parabolic_K = v_JK + a_K*t_in_b;
                
                joint_acceleration_rt_parabolic_K = ...
                    a_K * ones(size(absolute_time_span_parabolic_K));
                
                if generate_trajectory_bool == 1
                    qrt(segments_start_time_index(joint_index):...
                        (segments_start_time_index(joint_index) + ...
                        number_of_samples_in_segment_parabolic_K) - 1, ...
                        joint_index) = qrt_parabolic;
                    
                    qdotrt(segments_start_time_index(joint_index):...
                        (segments_start_time_index(joint_index) + ...
                        number_of_samples_in_segment_parabolic_K) - 1, ...
                        joint_index) = joint_velocity_rt_parabolic_K;
                    
                    qdotdotrt(segments_start_time_index(joint_index):...
                        (segments_start_time_index(joint_index) + ...
                        number_of_samples_in_segment_parabolic_K) - 1, ...
                        joint_index) = joint_acceleration_rt_parabolic_K;
                end

                absolute_time_segment(joint_index) = ...
                    absolute_time_segment(joint_index) + t_K;
                
                segments_start_time_index(joint_index) = ...
                    segments_start_time_index(joint_index) + ...
                    number_of_samples_in_segment_parabolic_K;

                figure(joint_index)
                subplot(3, 1, 1)
                plot(absolute_time_span_linear_JK, qrt_linear, ...
                    'LineWidth', 2);
                plot(absolute_time_span_parabolic_K, qrt_parabolic, ...
                    'LineWidth', 2);
                
                subplot(3, 1, 2)
                plot(absolute_time_span_linear_JK, ...
                    joint_velocity_rt_linear_JK, 'LineWidth', 2);
                plot(absolute_time_span_parabolic_K, ...
                    joint_velocity_rt_parabolic_K, 'LineWidth', 2);
                
                subplot(3, 1, 3)
                plot(absolute_time_span_linear_JK, ...
                    joint_acceleration_rt_linear_JK, 'LineWidth', 3);
                plot(absolute_time_span_parabolic_K, ...
                    joint_acceleration_rt_parabolic_K, 'LineWidth', 3);
            end
        end
            
    end
    number_of_via_points
    q_viapoints
    acceleration_at_via
    velocity_at_via
    blend_time_at_via
    linear_time_after_via
    absolute_time_segment
    via_point_index
    
    qrt_final = qrt(1:numel(time_span), 1:number_of_joints);
end