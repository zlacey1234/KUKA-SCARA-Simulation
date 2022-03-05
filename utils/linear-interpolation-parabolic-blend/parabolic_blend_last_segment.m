function [acceleration, blending_time, velocity] = ...
    parabolic_blend_last_segment(previous_point, end_point, ...
    desired_time_duration, desired_acceleration)
%%


acceleration = ...
    sign(previous_point - end_point) * abs(desired_acceleration);

if desired_acceleration == 0
    blending_time = 0;
else
    blending_time = desired_time_duration - ...
        sqrt(desired_time_duration^2 + ...
        2*(end_point - previous_point)/acceleration);
end

velocity = (end_point - previous_point)/(desired_time_duration - ...
    (1/2) * blending_time);

end
