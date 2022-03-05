%% MDL_SCARA_KUKA_KR_6_R700Z200 
% Summary: Create model of the KUKA KR 6 R700 Z200 (SCARA Arm). 
%
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
clear L
deg = pi/180;

% parameters SI Units: m, 
L1 = 0.185;      % 185.0mm or 0.185m
L2 = 0.425;      % 425.0mm or 0.425m
L3 = 0.275;      % 275.0mm or 0.275m
Ltool = 0.0694;  %  69.4mm or 0.0694m

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