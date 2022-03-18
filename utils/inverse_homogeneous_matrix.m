function inverse_matrix = ...
    inverse_homogeneous_matrix(homogeneous_matrix, jointType)
%% Function: Inverse_Homogeneous_Matrix
% Summary: Takes the inverse of a homogeneous transform matrix.
%
% INPUT: 
%   homogeneous_matrix: A (4 x 4) matrix representing a Homogeneous 
%                       Transformation matrix (contains a 3 x 3 rotation 
%                       matrix, 'R' as well as a 3 x 1 translation vector,
%                       'p')
% 
% OUTPUT: 
%   inverse_matrix: A (4 x 4) matrix that is the Inverse of the provided
%                   Homogeneous Transformation matrix. 
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%  
    expectedJointTypes = {'symbolic', 'numeric'};
    
    p = inputParser;
    
    addRequired(p, 'homogeneous_matrix', @(x)(true));
    addRequired(p, 'jointType', ...
        @(x)any(validatestring(x, expectedJointTypes)));
    
    parse(p, homogeneous_matrix, jointType);
    
    joint_type = validatestring(p.Results.jointType, expectedJointTypes);
    
    % Rotation Matrix 
    R = homogeneous_matrix(1:3, 1:3);

    % Translational Vector
    p = homogeneous_matrix(1:3, 4);
    
    if strcmp(joint_type, 'symbolic')
        inverse_matrix = sym(zeros(size(homogeneous_matrix)));
    else
        inverse_matrix = zeros(size(homogeneous_matrix));
    end
    
    inverse_matrix(1:3, 1:3) = inv(R);
    inverse_matrix(1:3, 4) = -inv(R)*p;
    inverse_matrix(4,4) = 1;
end