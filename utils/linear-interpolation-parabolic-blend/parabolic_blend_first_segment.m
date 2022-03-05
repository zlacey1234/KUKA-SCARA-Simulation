function [acceleration, blending_time, velocity] = ...
    parabolic_blend_first_segment(initial_point, goal_point, ...
    desired_time_duration, desired_acceleration)
%% Parabolic Blend First Segment
% Summary: The parabolic_blend_first_segment function calculates the
%          acceleration (theta_dotdot_{1}), velocity (theta_dot_{1_2}), 
%          and the blend time (t_{1}) of the First Segment. 
%          
% INPUTS:
%   initial_point:          This is the initial via point (Via Point 1) 
%                           for the First Segment of Linear with Parabolic 
%                           Blend Trajectory Generation Method.(theta_{1})
%
%   goal_point:             This is the goal via point (Via Point 2) 
%                           for the First Segment of Linear with Parabolic 
%                           Blend Trajectory Generation Method.(theta_{2})
% 
%   desired_time_duration:  The desired duration of the First Linear with 
%                           Parabolic Blend Segment.(t_d{1_2}) [units:
%                           seconds]
%
%   desired_acceleration:   The desired acceleration at the initial via
%                           point (Via Point 1). Acceleration constraint.
%                           (theta_dotdot_{1})
%   
% OUTPUTS:
%   acceleration:           The acceleration (with +- sign) at the initial 
%                           via point (Via Point 1).(theta_dotdot_{1})
%
%   blend_time:             The time in which it takes to blend (parabolic
%                           section of the trajectory).(t_{1}) 
%
%   velocity:               The velocity at the initial via point (Via
%                           Point 1). Velocity in the linear section of 
%                           the trajectory.(theta_dot_{1_2})
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
acceleration = ...
    sign(goal_point - initial_point) * abs(desired_acceleration);

if desired_acceleration == 0
    blending_time = 0;
elseif acceleration == 0
    blending_time = 0;
else
    blending_time = desired_time_duration - ...
        sqrt(desired_time_duration^2 + ...
        2*(initial_point - goal_point)/acceleration);
end

velocity = (goal_point - initial_point)/(desired_time_duration - ...
    (1/2) * blending_time);
end