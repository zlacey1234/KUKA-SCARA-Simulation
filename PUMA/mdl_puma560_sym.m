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
            
% Define the symbolic variables for the Dynamics of the PUMA 560. This
% includes Mass and Inertia Matrices and assumes they are
% real-numbers (not complex-numbers).
m1 = sym('m1', 'real');
m2 = sym('m2', 'real');
m3 = sym('m3', 'real');
m4 = sym('m4', 'real');
m5 = sym('m5', 'real');
m6 = sym('m6', 'real');
mtool = sym('mtool', 'real');

px_c1 = sym('px_c1', 'real');
py_c1 = sym('py_c1', 'real');
pz_c1 = sym('pz_c1', 'real');

px_c2 = sym('px_c2', 'real');
py_c2 = sym('py_c2', 'real');
pz_c2 = sym('pz_c2', 'real');

px_c3 = sym('px_c3', 'real');
py_c3 = sym('py_c3', 'real');
pz_c3 = sym('pz_c3', 'real');

I_xx_c1 = sym('I_xx_c1', 'real');
I_yy_c1 = sym('I_yy_c1', 'real');
I_zz_c1 = sym('I_zz_c1', 'real');

I_xx_c2 = sym('I_xx_c2', 'real');
I_yy_c2 = sym('I_yy_c2', 'real');
I_zz_c2 = sym('I_zz_c2', 'real');

I_xx_c3 = sym('I_xx_c3', 'real');
I_yy_c3 = sym('I_yy_c3', 'real');
I_zz_c3 = sym('I_zz_c3', 'real');

I_xx_c4 = sym('I_xx_c4', 'real');
I_yy_c4 = sym('I_yy_c4', 'real');
I_zz_c4 = sym('I_zz_c4', 'real');

I_xx_c5 = sym('I_xx_c5', 'real');
I_yy_c5 = sym('I_yy_c5', 'real');
I_zz_c5 = sym('I_zz_c5', 'real');

I_xx_c6 = sym('I_xx_c6', 'real');
I_yy_c6 = sym('I_yy_c6', 'real');
I_zz_c6 = sym('I_zz_c6', 'real');

L = [
    RevoluteMDH('alpha', 0.0, ...
                'a', 0.0, ...
                'd', 0.0, ...
                'm', m1, ...
                'r', [px_c1, py_c1, pz_c1], ...
                'I', [I_xx_c1, I_yy_c1, I_zz_c1, 0, 0, 0]);
                
    RevoluteMDH('alpha', sym(-pi/2), ...
                'a', 0.0, ...
                'd', 0.0, ...
                'm', m2, ...
                'r', [px_c2, py_c2, pz_c2], ...
                'I', [I_xx_c2, I_yy_c2, I_zz_c2, 0, 0, 0]);
                
    RevoluteMDH('alpha', 0.0, ...
                'a', a2, ...
                'd', d3, ...
                'm', m3, ...
                'r', [px_c3, py_c3, pz_c3], ...
                'I', [I_xx_c3, I_yy_c3, I_zz_c3, 0, 0, 0]);
                
    RevoluteMDH('alpha', sym(-pi/2), ...
                'a', a3, ...
                'd', d4, ...
                'm', m4, ...
                'r', [0.0, 0.0, 0.0], ...
                'I', [I_xx_c4, I_yy_c4, I_zz_c4, 0, 0, 0]);
    
    RevoluteMDH('alpha', sym(pi/2), ...
                'a', 0.0, ...
                'd', 0.0, ...
                'm', m5, ...
                'r', [0.0, 0.0, 0.0], ...
                'I', [I_xx_c5, I_yy_c5, I_zz_c5, 0, 0, 0]);
                
    RevoluteMDH('alpha', sym(-pi/2), ...
                'a', 0.0, ...
                'd', 0.0, ...
                'm', m6, ...
                'r', [0.0, 0.0, 0.0], ...
                'I', [I_xx_c6, I_yy_c6, I_zz_c6, 0, 0, 0]);
    ];

puma_560 = SerialLink(L, 'name', 'PUMA', 'tool', T_wrist_tool);
