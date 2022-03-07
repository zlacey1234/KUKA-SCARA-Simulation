%% MDL_SCARA_SYMS
% Summary: Create model of the(SCARA Arm) (Represented Symbolically).
%
% Applicable to the Following Robots:
%      KUKA KR 6 R700 Z200 Robotic Arm 
%
%      Mitsubishi RH-3FRH5515 Robotic Arm
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
clear L
deg = pi/180;

% Define the symbolic variables for Link Lengths and assume they are 
% real-numbers (not complex-numbers)
L1 = sym('L1', 'real');
L2 = sym('L2', 'real');
L3 = sym('L3', 'real');
Ltool = sym('Ltool', 'real');

T_wrist_tool = [1  0  0  0;...
                0  1  0  0;...
                0  0  1  -Ltool;...
                0  0  0  1];

L = [
    RevoluteMDH('alpha', 0.0, ...
    'a', 0.0, ...
    'd', L1, ...
    'qlim', [-170 170]*deg);
    
    RevoluteMDH('alpha', 0.0, ...
    'a', L2, ...
    'd', 0.0, ...
    'qlim', [-145 145]*deg);
    
    RevoluteMDH('alpha', 0.0, ...
    'a', L3, ...
    'd', 0.0, ...
    'qlim', [-360 360]*deg);
    
    PrismaticMDH('qlim', [-0.150 0])
    ];

scara = SerialLink(L, 'name', 'SCARA', 'tool', T_wrist_tool);