function [acceleration_at_K, blend_time_at_K, velocity_KL] = ...
    parabolic_blend_mid_points(initial_point, mid_point, ...
    goal_point, desired_time_duration_JK, desired_time_duration_KL, ...
    desired_acceleration, via_point_index_K, number_of_via_points)
%% Parabolic Blend Mid Segments
% Summary: The parabolic_blend_mid_points function calculates the
%          acceleration (theta_dotdot_{K}), velocity (theta_dot_{K_L}), 
%          and the blend time (t_{K}) of the Mid Segment. 
%
% INPUTS:
%   initial_point:          This is the initial via point (Via Point J) 
%                           for the Mid Segments of Linear with Parabolic 
%                           Blend Trajectory Generation Method.(theta_{J})
%    
%   mid_point:              This is the middle via point (Via Point K) 
%                           for the Mid Segments of Linear with Parabolic 
%                           Blend Trajectory Generation Method.(theta_{K})
% 
%   goal_point:             This is the goal via point (Via Point L) 
%                           for the First Segment of Linear with Parabolic 
%                           Blend Trajectory Generation Method.(theta_{L})
% 
%   desired_time_duration_JK:
%                           The desired duration of the first half of the 
%                           Linear with Parabolic Blend Trajectory (from 
%                           Via Point J to Via Point K).(t_d{J_K}) [units:
%                           seconds]
%
%   desired_time_duration_JK:
%                           The desired duration of the second half of the 
%                           Linear with Parabolic Blend Trajectory (from 
%                           Via Point K to Via Point L).(t_d{K_L}) [units:
%                           seconds]
%   desired_acceleration:   The desired acceleration at the middle via
%                           point (Via Point K).Acceleration constraint. 
%                           (theta_dotdot_{K})
% 
%   via_point_index_K:      The current Via Point Index. 
%   
%   number_of_via_points:   Number of Via Points. 
%
% OUTPUTS:
%   acceleration_at_K:      The acceleration (with +- sign) at the middle 
%                           via point (Via Point K).(theta_dotdot_{K})
%
%   blend_time_at_K:        The time in which it takes to blend (parabolic
%                           section of the trajectory).(t_{1}) 
%
%   velocity_KL:            The velocity at the initial via point (Via
%                           Point 1). Velocity in the linear section of 
%                           the trajectory.(theta_dot_{1_2})
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%

if (via_point_index_K - 1) == 1 % First Segment
    [acceleration_1, blend_time_1, velocity_JK] = ...
        parabolic_blend_first_segment(initial_point, mid_point, ...
        desired_time_duration_JK, desired_acceleration);
else
    velocity_JK = (mid_point - initial_point)/desired_time_duration_JK;
end

if (via_point_index_K + 1) == number_of_via_points % Last Segment
    [acceleration_n, blend_time_n, velocity_KL] = ...
        parabolic_blend_last_segment(mid_point, goal_point, ...
        desired_time_duration_KL, desired_acceleration);
else
    velocity_KL = (goal_point - mid_point)/desired_time_duration_KL;
end

acceleration_at_K = sign(velocity_KL - velocity_JK) * desired_acceleration;

if desired_acceleration == 0
    blend_time_at_K = 0;
elseif acceleration_at_K == 0
    blend_time_at_K = 0;
else
    blend_time_at_K = (velocity_KL - velocity_JK)/acceleration_at_K;
end 

end