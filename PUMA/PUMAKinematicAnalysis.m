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
mdl_puma560_sym;

%% Forward Kinematics 

% Define the symbolic variables for the Joints and assume they are
% real-numbers (not complex-numbers)
% th1 = sym('th1', 'real');
% th2 = sym('th2', 'real');
% th3 = sym('th3', 'real');
% th4 = sym('th4', 'real');
% th5 = sym('th5', 'real');
% th6 = sym('th6', 'real');

syms th1 th2 th3 th4 th5 th6

q_symbolic = [th1 th2 th3 th4 th5 th6];

% Calculate and Display the Homogeneous Transformation Matrices that
% represent the Forward Kinematics of the Robotic Arm 
display_forward_kinematics(puma_560, q_symbolic);
