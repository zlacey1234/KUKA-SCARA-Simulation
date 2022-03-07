function [q_trajectory, infeasible_ik_boolean] = ...
    ik_scara_kuka_kr_6_r700z200(robot, T_wrist, T_wrist_tool, ...
    angleType, varargin)
%% Function: ik_scara_kuka_kr_6_r700z200
% Summary: The ik_scara_kuka_kr_6_r700z200 function takes the inverse 
%          kinematics of the KUKA KR 6 R700 Z200
%   
% INPUTS:
%   robot:     The robot object that was created using Peter Corke's
%              Robotics Toolbox in MATLAB (using the SerialLink Object
%              Class).
%
%   T_wrist:    A matrix containing the pose (position and orientation) of
%               the wrist in the Homogeneous Transformation 
%               Representation. 
%
%   T_wrist_tool: The Transformation is specifically the Homogeneous 
%                 Transformation of the tool (Center point between the 
%                 finger tips of the Gripper) w.r.t the Wrist Frame {W} of 
%                 the KUKA KR 6 SCARA Arm. The matrix can be described 
%                 with (4 x 4) matrix for a Single Homogeneous
%                 Transformation or as (4 x 4 x N) matrix where we 
%                 describe N sets of Homogeneous matrices (these can be a 
%                 sequence to form a trajectory with N trajectory points). 
%
%   'angleType', {'deg', 'rad'}:         This allows the user to specify
%                                        what angle measurements type to
%                                        return the joint trajectory in. 
%
% OPTIONAL INPUTS: 
%   'elbowOption', {'up', 'down'}:       This is the option to specify if
%                                        the ik solution for the elbow
%                                        joint (theta2) since the ik has
%                                        two possible solutions for the
%                                        elbow joint. 
%   
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
    % Default Options
    defaultElbow = 'up';              % theta2 joint
    expectedElbow = {'up', 'down'};   % Possible Elbow Options
    expectedAngleTypes = {'deg', 'rad'};
    
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'T_tool', @(x)(true));
    addRequired(p, 'T_wrist_tool', @(x)(true));
    addRequired(p, 'angleType', ...
        @(x) any(validatestring(x, expectedAngleTypes)));
    addOptional(p, 'elbowOption', defaultElbow, ...
        @(x) any(validatestring(x, expectedElbow)));
    
    parse(p, robot, T_wrist, T_wrist_tool, angleType, varargin{:});
    
    elbow = validatestring(p.Results.elbowOption, expectedElbow);
    angle_type = validatestring(p.Results.angleType, expectedAngleTypes);
    
    tolerance = 1e-6;
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % Unpack the Homogenous Tool Point (or Trajectory)
    T_wrist = SE3(T_wrist);
    
    % Number of Trajectory Points
    number_of_trajectory_points = length(T_wrist);
    
    % Initialize Joint Space Matrix (N x 5) matrix where N is the number 
    % of trajectory points. 
    q_trajectory = zeros(number_of_trajectory_points, number_of_joints);
    
    % Infeasible Inverse Kinematics Flag (0: Valid, 1: Invalid)
    infeasible_ik_boolean = 0;
    
    % For each trajectory point
    for trajectory_point_index = 1:number_of_trajectory_points
        
        % Unpack the Homogeneous Transformation (4 x 4) matrix of the tool
        % w.r.t the Base Frame of the SCARA Arm.
        T_tool_current = T_wrist(trajectory_point_index);
        
        % Convert back to a Matrix Form SE3 ---> Matrix in MATLAB
        T_wrist_current_matrix = T_tool_current.T * inv(T_wrist_tool);
        
        %% Analytical Solution to Inverse Kinematics for SCARA
        % Unpack the Robot Object DH-Parameters
        L1 = L(1).d;
        L2 = L(2).a;
        L3 = L(3).a;
        
        % Upack Current Homogenous Tool Pose
        % Position of the tool (XYZ) w.r.t the Base Frame {B} of SCARA
        Px = T_wrist_current_matrix(1,4);
        Py = T_wrist_current_matrix(2,4);
        Pz = T_wrist_current_matrix(3,4);
        
        r11 = T_wrist_current_matrix(1,1);
        r21 = T_wrist_current_matrix(2,1);
        
        % Solve for theta2
        b = (Px^2 + Py^2 - L3^2 - L2^2)/(2*L2*L3);
        
        if (abs(b - 1.000) <= tolerance)
            theta2 = 0;
        elseif (abs(b + 1.000) <= tolerance)
            theta2 = pi;
        else
            if strcmp(elbow, 'up') 
                if isreal(sqrt(1 - b^2))
                    theta2 = atan2(sqrt(1 - b^2), b);
                else
                    theta2 = 0;
                    infeasible_ik_boolean = 1;
                end
            elseif strcmp(elbow, 'down')
                if isreal(sqrt(1 - b^2))
                    theta2 = atan2(-sqrt(1 - b^2), b);
                else
                    theta2 = 0;
                    infeasible_ik_boolean = 1;
                end
            else 
                print("Error! elbow variable: " + str(elbow))
            end
        end
        
        % Solve for theta1
        k1 = L3*cos(theta2) + L2;
        k2 = L3*sin(theta2);
        
        theta1 = atan2(Py, Px) - atan2(k2, k1);
        
        % Solve for theta3
        phi = atan2(r21, r11);
        
        theta3 = phi - theta1 - theta2;
        
        % Solve for d4
        d4 = Pz - (L1);
        
        % Convert to Revolute Joints to Degrees (rad --> deg)
        if strcmp(angle_type, 'deg') 
            theta1 = rad2deg(theta1); 
            theta2 = rad2deg(theta2);
            theta3 = rad2deg(theta3);
        end
        
        q_trajectory(trajectory_point_index,:) = ...
            [theta1, theta2, theta3, d4];
    end
end