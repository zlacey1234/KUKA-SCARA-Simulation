function J_num = ...
    jacobian_syms2numeric(robot, joints_numeric, angleType, J_sym)
%% Function: Jacobian_Numeric
% Summary: This function takes the Kinematic Jacobian Matrix (represented
%          Symbolically) and substitutes the joints values into the obtain
%          the Kinematic Jacobian Matrix (represented Numerically). 
%
% Important Note: This function is useful when you must calculate the
%                 Kinematic Jacobian Matrix via the Force or Velocity
%                 Propagation Methods. This is because rather than solving
%                 and propagating the Jacobian Iteratively at every joint
%                 step, you Solve for the Jacobian Symbolically using
%                 J_sym = jacobian_force_propagation() or 
%                 J_sym = jacobian_velocity_propagation() functions and 
%                 then at each joint step, you substitute the joint values 
%                 into the Symbolic Matrix (J_sym). This is not very
%                 useful for the Explicit Method implementation. 
%
% Note On Jacobian Performance: (Ranking from Best to Worst)
%   1.      Use symbolic methods to find a the Kinematic Jacobian, then 
%           implement the method as a jacobian function (i.e.,
%           jacobian_scara_rh3frh5515() is implemented. This reduces the 
%           computational time the most.
%
%   2.      Use the explicit method directly to solve for the Kinematic
%           Jacobian (i.e., use 
%           J_i = jacobian_explicit(robot, q_i, 'deg', 'numeric') right
%           after using ik_scara_rh3frh5515() function). 
% 
%   3.      If you use the J_sym = jacobian_force_propagation() and you
%           need to J_sym, you should use this jacobian_syms2numeric()
%           function. This takes quite longer computationally. Same if you
%           implement J_sym = jacobian_velocity_propagation() or 
%           J_sym = jacobian_explicit().
%
%   4.      Using jacobian_velocity_propagation() or 
%           jacobian_force_propagation() is the slowest since it goes
%           through the propagation for every attempted calculation. So if
%           you calculate the Jacobian Matrix for 100 trajectory points,
%           then for each point, it propagates and solves the Jacobian.
%           This becomes very computationally intensive (and is not very
%           efficient).
%
% INPUTS:
%   robot: @SerialLink Object created using the Robotics Toolbox by Peter
%          Corke.
%
%   joints_numeric: Numeric Joint Values.  
%
%   'angleType', {'deg', 'rad'}:         This allows the user to specify
%                                        what angle measurements type to
%                                        return the joint trajectory in. 
%
%   J_sym: A (6 x N) matrix that represents the kinematic Jacobian of the
%          Serial Robot. N is the number of active joints in the kinematic
%          serial chain. Elements in this matrix may be of symbolic 
%          functions, which would be a function of the joint values.
%
% OUTPUT:
%   J_num: A (6 x N) matrix that represents the kinematic Jacobian of the
%          Serial Robot. N is the number of active joints in the kinematic
%          serial chain. Each element in the matrix should be a numeric
%          value [datatype: double]. 
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
    % Expected Input Value
    expectedAngleTypes = {'deg', 'rad'};

    p = inputParser;

    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'joints_numeric', @(x)(true));
    addRequired(p, 'angleType', ...
        @(x) any(validatestring(x, expectedAngleTypes)));
    addRequired(p, 'J_sym', @(x)(true));
    
    parse(p, robot, joints_numeric, angleType, J_sym);

    angle_type = validatestring(p.Results.angleType, expectedAngleTypes);

    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();

    for joint_index = 1:number_of_joints
        if L(joint_index).isrevolute
            % If the numeric joint values are in degrees, and the joint is
            % a revolute joint, convert to radians in order to get the 
            % proper numeric Kinematic Jacobian Matrix (expects radian 
            % values)
            if strcmp(angle_type, 'deg')
                joints_numeric(joint_index) = ...
                    deg2rad(joints_numeric(joint_index));
            end
        end
        joint_string = ['q' num2str(joint_index)];
        J_sym = subs(J_sym, joint_string, joints_numeric(joint_index));
    end

    J_num = double(J_sym);
end