function q_trajectory_updated = ...
    deg2rad_q_trajectory(robot, q_trajectory)
%% Function deg2rad_q_trajectory
% Summary: Takes a trajectory (Joint Space trajectory) and based on the
%          serial robot, it converts the joint angles from degrees to
%          radians. 
%
% INPUT: 
%   robot: 
%
%   q_trajectory: A (N x n) matrix where N is the number of trajectory
%                 points and n is the number of joints that define the
%                 serial chain of the robot. Each row represents the joint
%                 values of the serial robot at that instance of the
%                 trajectory. [units: degrees]
%
% OUTPUT: 
%   q_trajectory_updated: A (N x n) matrix similar to q_trajectory but it
%                         contains the joint values converted to radians.
%                         [units: radian]
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
    % Link Object
    L = robot.links();
    
    % Number of Joints (Prismatic or Revolute)
    number_of_joints = robot.n();
    
    % Initalize the updated joint trajectory
    q_trajectory_updated = zeros(size(q_trajectory));
    
    % For each joint
    for joint_index = 1:number_of_joints
        % All values in a specific joint (Entire Joint Column)
        joint_trajectory_current = q_trajectory(:, joint_index);
        
        % If the Joint is Revolute 
        if L(joint_index).isrevolute()
            % Convert degree to radian
            q_trajectory_updated(:, joint_index) = ...
                deg2rad(joint_trajectory_current);
        else
            % No conversion for Prismatic Joints
            q_trajectory_updated(:, joint_index) = ...
                joint_trajectory_current;
        end
    end
end