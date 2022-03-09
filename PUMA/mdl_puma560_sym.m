%% MDL_PUMA560_SYMS
% Summary: Creates a model of the PUMA 560 Robotic Arm (Represented
%          Symbolically).
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
clear L
deg = pi/180;

% Define the symbolic variables for the Kinematics of the PUMA 560. This
% includes Link Lengths and Offset Lengths and assumes they are
% real-numbers (not complex-numbers). Symbolic variables for offset
% DH-Parameters.
a2 = sym('a2', 'real');
d3 = sym('d3', 'real');
a3 = sym('a3', 'real');
d4 = sym('d4', 'real');

% Transformation from the wrist to the tool 
Px_6_tool = sym('Px_6_tool', 'real');
Pz_6_tool = sym('Pz_6_tool', 'real');

T_wrist_tool = [1  0  0  Px_6_tool;...
                0  1  0  0;...
                0  0  1  Pz_6_tool;...
                0  0  0  1];

L = [
    RevoluteMDH('alpha', 0.0, ...
                'a', 0.0, ...
                'd', 0.0);
                
    RevoluteMDH('alpha', sym(-pi/2), ...
                'a', 0.0, ...
                'd', 0.0);
                
    RevoluteMDH('alpha', 0.0, ...
                'a', a2, ...
                'd', d3);
                
    RevoluteMDH('alpha', sym(-pi/2), ...
                'a', a3, ...
                'd', d4);
    
    RevoluteMDH('alpha', sym(pi/2), ...
                'a', 0.0, ...
                'd', 0.0);
                
    RevoluteMDH('alpha', sym(-pi/2), ...
                'a', 0.0, ...
                'd', 0.0)
    ];

puma_560 = SerialLink(L, 'name', 'PUMA', 'tool', T_wrist_tool);
