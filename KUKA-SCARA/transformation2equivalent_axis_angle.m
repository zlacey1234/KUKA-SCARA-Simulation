function state = transformation2equivalent_axis_angle(T_S_tool)
    
    %%
    
    tolerance = 1e-3;
    
    R_S_tool = T_S_tool(1:3, 1:3);
    
    position_S_tool = T_S_tool(1:3, 4);
    
%     axis_angle = rotm2axang(R_S_tool);
    
    Symmetric = R_S_tool - R_S_tool';
    
    r11 = R_S_tool(1,1);
    r12 = R_S_tool(1,2);
    r13 = R_S_tool(1,3);
    
    r21 = R_S_tool(2,1);
    r22 = R_S_tool(2,2);
    r23 = R_S_tool(2,3);
    
    r31 = R_S_tool(3,1);
    r32 = R_S_tool(3,2);
    r33 = R_S_tool(3,3);
    
    theta = acos((r11 + r22 + r33 - 1)/2);
    
    if abs(theta - 0) <= tolerance
        K_hat = [0; 0; 0];
    elseif Symmetric <= ones(3,3)*tolerance
        % 
        eigen_vector = eig(R_S_tool);
        K_hat_check = eigen_vector - ones(3, 1);
        
        K_hat = abs(K_hat_check) < tolerance;
    else
        K_hat = (1/(2*sin(theta))) * [(r32 - r23); ...
                                      (r13 - r31); ...
                                      (r21 - r12)];
    end
%     K_hat = axis_angle(1:3)';
%     theta = axis_angle(4);
    K = theta*K_hat;
    
    state = [position_S_tool ; ...
             K];
end