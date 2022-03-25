function EoM = Newton_Euler_Equations_sym(robot, dof)
%% Function: Newton_Euler_Equations_Sym
% Summary:
%
% INPUT: 
%
% OUTPUT:
% 
%%
    % Parser
    p = inputParser;
    
    addRequired(p, 'robot', @(x)(true));
    addRequired(p, 'dof', @(x)(true));
    
    parse(p, robot, dof);
    
    % Gravitational Acceleration [units: m/(s^2)]
    g = sym('g', 'real');
    
    % Unpack the Robot Object
    L = robot.links;
    
    % Number of Joints 
    number_of_joints = robot.n();
    
    % Joint (joint)
    joints = sym('q%d', [1 number_of_joints], 'real');
    
    % Joint Velocity (joint-dot)
    dot_joint = sym('q%ddot', [1 number_of_joints], 'real');
    
    % Joint Acceleration (joint-dotdot)
    dotdot_joint = sym('q%ddotdot', [1 number_of_joints], 'real');
    
    if dof ~= number_of_joints
        number_of_joints = dof
        tool_bool = 0
    end
    
    % Linear and Angular Velocity 
    v_i_i = sym(zeros(3, number_of_joints + 1));
    w_i_i = sym(zeros(3, number_of_joints + 1));
    
    % Linear and Angular Acceleration 
    vdot_i_i = sym(zeros(3, number_of_joints + 1));
    wdot_i_i = sym(zeros(3, number_of_joints + 1));
    
    vdot_i_Ci = sym(zeros(3, number_of_joints + 1));
    
    vdot_i_i(:, 1) = [0.0;  0.0;  g];
    
    joint_0_wrist_span = 1:1:number_of_joints;
    
    % Forces (at Center of Mass)
    F_i_i = sym(zeros(3, number_of_joints + 1));
    
    % Torques (at Center of Mass)
    N_i_i = sym(zeros(3, number_of_joints + 1));
    
    
    % Homogeneous Transformation Matrix of the Wrist w.r.t the 
    % Body Frame {0} 
    T_0_N = simplify(robot.A(joint_0_wrist_span,... 
        joints));
    T_0_N_matrix = T_0_N.T;
    
    % Unpack the Translation (P_0_N) and Rotation (R_0_N) from the
    % Homogeneous Transformation Matrix
    P_0_N = T_0_N_matrix(1:3, 4);
    R_0_N = T_0_N_matrix(1:3, 1:3);
    
    R_iprev_icurr_store = sym(zeros(3, 3, number_of_joints));
    
%     T_iprev_tool = simplify(robot.A(number_of_joints, joints));
%         
%     T_iprev_tool_matrix = T_iprev_tool.T;
%         
%     R_iprev_tool = T_iprev_tool_matrix(1:3, 1:3);
%         
%     R_iprev_icurr_store(:, :, number_of_joints) = R_iprev_tool;
    
    for joint_index = 1:number_of_joints
        % iprev is equivalent to (joint_index) or (i) [DH: i-1]
        % icurr is equivalent to (joint_index + 1) or (i + 1) [DH: i]
        T_iprev_icurr = simplify(robot.A(joint_index, joints));
        
        T_iprev_icurr_matrix = T_iprev_icurr.T;
        
        R_iprev_icurr = T_iprev_icurr_matrix(1:3, 1:3);
        
        R_iprev_icurr_store(:, :, joint_index) = R_iprev_icurr;
        
        R_icurr_iprev = transpose(R_iprev_icurr);
        
        P_iprev_icurr = T_iprev_icurr_matrix(1:3, 4)
        
        P_icurr_CMi = L(joint_index).r
        
        % Mass of current link (m_icurr
        m_icurr = L(joint_index).m;
        
        I_icurr = L(joint_index).I;
        if L(joint_index).isrevolute
            % Angular Velocity
            w_i_i(:, joint_index + 1) = ...
                simplify(R_icurr_iprev * w_i_i(:, joint_index) + ...
                dot_joint(joint_index) * R_iprev_icurr(:, 3));
            
            % Linear Velocity
            v_i_i(:,joint_index + 1) = ...
                simplify(collect(R_icurr_iprev * ...
                (cross(w_i_i(:, joint_index), P_iprev_icurr) + ...
                v_i_i(:,joint_index)), dot_joint));
            
            % Angular Acceleration
            wdot_i_i(:, joint_index + 1) = ...
                simplify(R_icurr_iprev * wdot_i_i(:, joint_index) + ...
                R_icurr_iprev * cross(w_i_i(:, joint_index), ...
                dot_joint(joint_index) * R_iprev_icurr(:, 3)) + ...
                dotdot_joint(joint_index) * R_iprev_icurr(:, 3));
            
            % Linear Acceleration
            vdot_i_i(:, joint_index + 1) = ...
                simplify(collect(R_icurr_iprev * ...
                ((cross(wdot_i_i(:, joint_index), P_iprev_icurr)) + ...
                (cross(w_i_i(:, joint_index), ...
                cross(w_i_i(:, joint_index), P_iprev_icurr))) + ... 
                vdot_i_i(:, joint_index))));
            
            vdot_i_Ci(:, joint_index + 1) = ...
                simplify(...
                (cross(wdot_i_i(:, joint_index + 1), P_icurr_CMi')) + ...
                (cross(w_i_i(:, joint_index + 1), ...
                (cross(w_i_i(:, joint_index + 1), P_icurr_CMi')))) + ...
                vdot_i_i(:, joint_index + 1))
            
            % Forces (at Center of Mass)
            F_i_i(:, joint_index) = ...
                m_icurr * vdot_i_Ci(:, joint_index + 1)
            
            % Torque (at Center of Mass)
            N_i_i(:, joint_index) = ...
                I_icurr * wdot_i_i(:, joint_index + 1) + ...
                cross(w_i_i(:, joint_index + 1), ...
                I_icurr * w_i_i(:, joint_index + 1))
            
        else
            % Angular Velocity
            w_i_i(:,joint_index + 1) = ...
                simplify(R_icurr_iprev * w_i_i(:, joint_index));
            
            % Linear Velocity
            v_i_i(:,joint_index + 1) = ...
                collect(R_icurr_iprev * ...
                (cross(w_i_i(:, joint_index), P_iprev_icurr) + ...
                v_i_i(:,joint_index) + ...
                [0; 0; dot_joint(joint_index)]), ...
                dot_joint);
        end
    end
    
    % Force/Torque applied by End-Effector 
    force_torque = sym('ft%d', [1 6], 'real');
    
    % Force/Torques on the Joints
    f_i_i = sym(zeros(3, number_of_joints + 1));
    n_i_i = sym(zeros(3, number_of_joints + 1));
    
    % Initial Force applied by the End-Effector.
    f_i_i(:, number_of_joints + 1) = ...
        [force_torque(1); force_torque(2); force_torque(3)];
    
    n_i_i(:, number_of_joints + 1) = ...
        [force_torque(4); force_torque(5); force_torque(6)];
    
    joint_ft = sym(zeros(number_of_joints, 1));
    
    %% Inward Iteration
    for joint_index = number_of_joints:-1:1
        
        P_icurr_CMi = L(joint_index).r;
        
        % Following Joint (joint_index + 1) or (i + 1) [DH: tool].
        % This means the Force/Torque at the Joint considered is the
        % Force/Torque applied at the Tool. 
        if joint_index == number_of_joints && tool_bool == 1
            
            T_icurr_tool = robot.tool
            
            T_icurr_tool_matrix = T_icurr_tool.T;
        
            R_icurr_tool = T_icurr_tool_matrix(1:3, 1:3);
        
            P_icurr_tool = T_icurr_tool_matrix(1:3, 4);
            
            f_i_i(:, joint_index) = ...
                R_icurr_tool * f_i_i(:, joint_index + 1) + ...
                F_i_i(:, joint_index);
            
            n_i_i(:, joint_index) = ... 
                N_i_i(:, joint_index) + ...
                R_icurr_tool * n_i_i(:, joint_index + 1) + ...
                cross(P_icurr_CMi', F_i_i(:, joint_index)) + ...
                cross(P_icurr_tool, ...
                R_icurr_tool * f_i_i(:, joint_index + 1));
            
        else
            T_iprev_icurr = simplify(robot.A(joint_index + 1, joints));
        
            T_iprev_icurr_matrix = T_iprev_icurr.T;
        
            R_iprev_icurr = T_iprev_icurr_matrix(1:3, 1:3);
        
            P_iprev_icurr = T_iprev_icurr_matrix(1:3, 4);
            
            f_i_i(:, joint_index) = ...
                R_iprev_icurr * f_i_i(:, joint_index + 1) + ...
                F_i_i(:, joint_index);
            
            n_i_i(:, joint_index) = ... 
                N_i_i(:, joint_index) + ...
                R_iprev_icurr * n_i_i(:, joint_index + 1) + ...
                cross(P_icurr_CMi', F_i_i(:, joint_index)) + ...
                cross(P_iprev_icurr, ...
                R_iprev_icurr * f_i_i(:, joint_index + 1));
        end
        
        
        
    end
    
    for joint_index = number_of_joints:-1:1
        R_iprev_icurr = R_iprev_icurr_store(:, :, joint_index);
        if L(joint_index).isrevolute
%             disp('Revolute')
            joint_ft(joint_index) = ...
                transpose(n_i_i(:, joint_index)) * R_iprev_icurr(:, 3);
        else
%             disp('Prismatic')
            joint_ft(joint_index) = ...
                transpose(f_i_i(:, joint_index)) * R_iprev_icurr(:, 3);
        end 
    end
    
    joint_ft = simplify(joint_ft)
    f_i_i = simplify(f_i_i)
    n_i_i = simplify(n_i_i)
    
    EOM = sym(zeros(number_of_joints, 1));
    w_i_i
%     v_i_i
%     
    wdot_i_i
    vdot_i_i
    
    R_iprev_icurr_store
%     
    Mass_Inertia_Matrix = equationsToMatrix(joint_ft, dotdot_joint)
    
    simplify(Mass_Inertia_Matrix)
    EoM = joint_ft;
end