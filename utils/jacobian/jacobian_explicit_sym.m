function J_sym = jacobian_explicit_sym(robot)
%% Function: Jacobian_Explicit_Sym
% Summary: This is a function that iteratively solves for the Kinematic
%          Jacobian Matrix using the Explicit Method. Returns the 
%          Kinematic Jacobian Matrix (represented Symbolically).
%
% Important Note:
%   This function should only really be used to obtain the symbolic
%   representation of the Jacobian Matrix. It may be useful to then create
%   a custom jacobian function specific for the robot (i.e., create a
%   method similar to jacobian_scara_rh3frh5515 example but for the robot
%   being developed for). This custom jacobian will likely be more
%   computationally efficient to integrate to trajectory generation or
%   control methods that may require repeated calculations of the Jacobian
%   Matrix (at various instances in time). 
%
% INPUTS: 
%   robot: @SerialLink Object created using the Robotics Toolbox by Peter
%          Corke.
%
% OUTPUT:
%   J_sym: A (6 x N) matrix that represents the kinematic Jacobian of the
%          Serial Robot. N is the number of active joints in the kinematic
%          serial chain. Returns the Kinematic Jacobian Matrix 
%          (represented Symbolically as a function of the joints_sym).
%          Where joints_sym = [q1 q2 ... qN]. 
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com    
%%   
    % Parser
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    
    parse(p, robot);

    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    J_sym = sym(zeros(6, number_of_joints));

    joints_sym = sym('q%d', [1 number_of_joints], 'real');
    
    joint_0_wrist_span = 1:1:number_of_joints;

    T_0_wrist = simplify(robot.A(joint_0_wrist_span,... 
        joints_sym));
    
    T_0_wrist_matrix = T_0_wrist.T;
    
    P_0_wrist = T_0_wrist_matrix(1:3, 4);
    
    for joint_index = 1:number_of_joints
        
        joint_0_i_span = 1:1:joint_index;
        
        T_0_i = simplify(robot.A(joint_0_i_span, joints_sym));
        
        T_0_i_matrix = T_0_i.T;
        
        P_0_i = T_0_i_matrix(1:3, 4);
        
        Z_0_i = T_0_i_matrix(1:3, 3);
        
        if L(joint_index).isrevolute
            Jv_0_i = cross(Z_0_i, (P_0_wrist - P_0_i));
            Jw_0_i = Z_0_i;
        else
            Jv_0_i = Z_0_i;
            Jw_0_i = zeros(3,1);
        end
        
        J_sym(:, joint_index) = [Jv_0_i; Jw_0_i];
        
    end

    J_sym = simplify(J_sym);
end