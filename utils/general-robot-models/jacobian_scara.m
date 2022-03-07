function J = jacobian_scara(robot, joints, angleType)
%% Function: Jacobian_Scara
%
% Applicable to the Following Robots:
%      KUKA KR 6 R700 Z200 Robotic Arm 
%
%      Mitsubishi RH-3FRH5515 Robotic Arm
    %% Unpack 
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % Unpack the Robot Object DH-Parameters
    L1 = L(1).d;
    L2 = L(2).a;
    L3 = L(3).a;
    
    % Unpack the Joints [units: radians]
    theta1 = joints(1);
    theta2 = joints(2); 
    theta3 = joints(3); 
    d4 = joints(4);

    if strcmp(angleType, 'deg')
        theta1 = deg2rad(theta1);
        theta2 = deg2rad(theta2);
        theta3 = deg2rad(theta3);
    end
    
    J = zeros(6, number_of_joints);
    
    J(1,1) = -L3 * sin(theta1 + theta2) - L2 * sin(theta1);
    J(1,2) = -L3 * sin(theta1 + theta2);
    
    
    J(2,1) = L3 * cos(theta1 + theta2) + L2 * cos(theta1);
    J(2,2) = L3 * cos(theta1 + theta2);
    
 
    J(3,4) = 1;
    
    
    J(6,1) = 1;
    J(6,2) = 1;
    J(6,3) = 1;
    
end