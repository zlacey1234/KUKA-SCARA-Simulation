function scara = mdl_scara_kuka_kr_6_r700z200(varargin)
%% Function: MDL_SCARA_KUKA_KR_6_R700Z200 
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

    defaultL1 = 0.185;        % L1:      185.0mm or 0.1850m
    defaultL2 = 0.425;        % L2:      425.0mm or 0.4250m
    defaultL3 = 0.275;        % L3:      275.0mm or 0.2750m
    defaultLtool = 0.0694;    % Ltool:    69.4mm or 0.0694m

    p = inputParser;
    validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);

    addOptional(p, 'L1', defaultL1, validScalarPosNum);
    addOptional(p, 'L2', defaultL2, validScalarPosNum);
    addOptional(p, 'L3', defaultL3, validScalarPosNum);
    addOptional(p, 'Ltool', defaultLtool, validScalarPosNum);

    parse(p, varargin{:});

    l_1 = p.Results.L1;
    l_2 = p.Results.L2;
    l_3 = p.Results.L3;
    l_tool = p.Results.Ltool;

    % parameters SI Units: m, 
    L1 = l_1;
    L2 = l_2;   
    L3 = l_3;      
    Ltool = l_tool;  

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
    
end