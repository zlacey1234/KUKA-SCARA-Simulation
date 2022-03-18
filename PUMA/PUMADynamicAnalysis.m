%% M-File: PUMAKinematicAnalysis 
% Summary: This m-file includes the Kinematic Analysis of the PUMA 560
%          Robotic Arm. The Kinematic Analysis includes the Forward
%          Kinematics, Inverse Kinematics, Kinematic Jacobian Matrix, and
%          Singularity Analysis. 
%
%   
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%%
% Clear the Workspace
clear 
close all 
clc

path_to_workspace = updatePaths();

% Load the PUMA 560 Model (Symbolic)
mdl_puma560_sym

EOM = Newton_Euler_Equations_sym(puma_560, 3)
