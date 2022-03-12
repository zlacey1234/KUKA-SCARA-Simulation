%% M-File: PUMASimulate
%
%
%%
% Clear the Workspace
clear 
close all 
clc

path_to_workspace = updatePaths();

path_to_CAD = [path_to_workspace, '\PUMA-CAD'];

% Load the PUMA 560 Model
puma = mdl_puma560;

%% 

% Transformation to Stationary Frame {S} w.r.t the Base Frame {0}
T_0_S = [1  0  0  0.000;...
         0  1  0  0.000;...
         0  0  1 -0.672;...
         0  0  0  1];

% Transformation to the Corner of the Cube w.r.t the Stationary Frame {S}
T_S_Cube = [1  0  0  0.500;...
            0  1  0  0.000;...
            0  0  1  0.000;...
            0  0  0  1];

% Transformation to the Corner of the Cube w.r.t the Base Frame {0}
T_0_Cube = T_0_S * T_S_Cube;


% T_tool_test = 
% 
% qrt_ = ik_puma560(puma, T_tool, 'rad')


qrt_test = [pi/2 0 0.907 pi/3 -pi/2 pi/4];

T_0_6_test = puma.A([1 2 3 4 5 6], qrt_test)

T_tool_test = T_0_6_test * puma.tool

qrt_ = ik_puma560(puma, T_tool_test, 'rad')


%%
clf
disp('Starting to Dispay')
disp(path_to_CAD)
ae = [200 30];
figure

plot_metal_cube(T_0_Cube)
hold on

figure_workspace = [-0.2 0.60 -0.2 0.4 -0.672 0.6];

puma.plot3d(qrt_test, 'view', ae, ...
    'path', path_to_CAD, ...
    'workspace', [-0.2 0.55 -0.2 0.4 -0.672 0.6]);