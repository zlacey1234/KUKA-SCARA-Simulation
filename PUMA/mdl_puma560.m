function puma = mdl_puma560(varargin)
%% Function: MDL_PUMA560
% Summary: Creates a model of the PUMA 560 Robotic Arm
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
    clear L
    deg = pi/180;
    
    % Default DH Parameters
    
    a2 = 0.4318;
    d3 = 0.1254;
    
    a3 = 0.0191;
    d4 = 0.4318;
    
    L = [
        RevoluteMDH('alpha', 0.0, ...
                    'a', 0.0, ...
                    'd', 0.0, ...
                    'qlim', [-160 160]*deg, ...
                    'm', 0.0, ...
                    'r', [0.0, 0.0, 0.0]);
                
        RevoluteMDH('alpha', -pi/2, ...
                    'a', 0.0, ...
                    'd', 0.0, ...
                    'qlim', [-223 43]*deg, ...
                    'm', 17.40, ...
                    'r', [0.068, 0.006, ]);
                    
        RevoluteMDH('alpha', 0.0, ...
                    'a', a2, ...
                    'd', d3, ...
                    'qlim', [-232 52]*deg, ...
                    'm', 4.80,...
                    'r', [0.0, 0.0687, 0.0140]);
                    
        RevoluteMDH('alpha', -pi/2, ...
                    'a', a3, ...
                    'd', d4, ...
                    'qlim', [-140 140]*deg, ...
                    'm', 0.82);
                    
        RevoluteMDH('alpha', pi/2, ...
                    'a', 0.0, ...
                    'd', 0.0, ...
                    'qlim', [-100 100]*deg, ...
                    'm', 0.);
                                
        RevoluteMDH('alpha', -pi/2, ...
                    'a', 0.0, ...
                    'd', 0.0, ...
                    'qlim', [-266 266]*deg)
        ];
    
    puma = SerialLink(L, 'name', 'PUMA')
    
end