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
[puma, puma_CAD_model] = mdl_puma560;

%% 

% Transformation to Stationary Frame {S} w.r.t the Base Frame {0}
T_0_S = [1  0  0  0.000;...
         0  1  0  0.000;...
         0  0  1 -0.672;...
         0  0  0      1];
     
T_S_0 = inverse_homogeneous_matrix(T_0_S, 'numeric');

% Transformation to the Corner of the Cube w.r.t the Stationary Frame {S}
T_S_Cube = [1  0  0  0.300;...
            0  1  0  0.400;...
            0  0  1  0.200;...
            0  0  0  1];

% Transformation to the Corner of the Cube w.r.t the Base Frame {0}
T_0_Cube = T_0_S * T_S_Cube;

% Transformation to the Initial Configuration w.r.t the Stationary Frame
% {S}
T_S_init_final = [0         0         1    0.4509; ...
                  0        -1         0    0.1254; ...
                  1         0         0    0.2402; ...
                  0         0         0         1];
              
% Transformation to the Initial Configuration w.r.t the Stationary Frame
% {S}
T_S_init_lift = [0         0         1    0.4509; ...
                 0        -1         0    0.1254; ...
                 1         0         0    0.4002; ...
                 0         0         0         1];

T_surface_tool_tip = [1      0      0    0.0050; ...
                      0      1      0    0.0000; ...
                      0      0      1    0.0000; ...
                      0      0      0         1];
                  
%% Letters on the Cube

% Letter 1: 'Z'

T_Cube_Z_1 = [0   0   1    0.0000; ...
              0  -1   0    0.1000; ...
              1   0   0    0.1000; ...
              0   0   0         1];
         
T_Cube_Z_2 = [0   0   1    0.1000; ...
              0  -1   0    0.1000; ...
              1   0   0    0.1000; ...
              0   0   0         1];
          
T_Cube_Z_3 = [0   0   1    0.0000; ...
              0  -1   0    0.0000; ...
              1   0   0    0.1000; ...
              0   0   0         1];

T_Cube_Z_4 = [0   0   1    0.1000; ...
              0  -1   0    0.0000; ...
              1   0   0    0.1000; ...
              0   0   0         1];
          
T_Cube_Z_5 = [0   0   1    0.1000; ...
              0  -1   0    0.0000; ...
              1   0   0    0.1500; ...
              0   0   0         1];
          
% Letter 2: 'B'

T_Cube_B_1 = [1   0   0    0.1000; ...
              0  -1   0    0.0000; ...
              0   0  -1    0.0500; ...
              0   0   0         1];
          
T_Cube_B_2 = [1   0   0    0.1000; ...
              0  -1   0    0.0750; ...
              0   0  -1    0.0500; ...
              0   0   0         1];
          
T_Cube_B_3 = [1   0   0    0.1000; ...
              0  -1   0    0.1000; ...
              0   0  -1    0.0250; ...
              0   0   0         1];

T_Cube_B_4 = [1   0   0    0.1000; ...
              0  -1   0    0.0750; ...
              0   0  -1    0.0000; ...
              0   0   0         1];

T_Cube_B_5 = [1   0   0    0.1000; ...
              0  -1   0    0.0000; ...
              0   0  -1    0.0000; ...
              0   0   0         1];

T_Cube_B_6 = [1   0   0    0.1000; ...
              0  -1   0    0.0000; ...
              0   0  -1    0.1000; ...
              0   0   0         1];
          
T_Cube_B_7 = [1   0   0    0.1000; ...
              0  -1   0    0.0750; ...
              0   0  -1    0.1000; ...
              0   0   0         1];
          
T_Cube_B_8 = [1   0   0    0.1000; ...
              0  -1   0    0.1000; ...
              0   0  -1    0.0750; ...
              0   0   0         1];
          
T_Cube_B_9 = [1   0   0    0.1000; ...
              0  -1   0    0.0750; ...
              0   0  -1    0.0500; ...
              0   0   0         1];

% Letter 3: 'L'

T_Cube_L_1 = [0   1   0    0.1000; ...
              1   0   0    0.1000; ...
              0   0  -1    0.1000; ...
              0   0   0         1];
          
T_Cube_L_2 = [0   1   0    0.1000; ...
              1   0   0    0.1000; ...
              0   0  -1    0.0000; ...
              0   0   0         1];  
          
T_Cube_L_3 = [0   1   0    0.0000; ...
              1   0   0    0.1000; ...
              0   0  -1    0.0000; ...
              0   0   0         1];   
          
%% Via Point Trajectory Representation

T_S_viapoints = zeros(4, 4, 1);

%% Letter 1: 'Z' 

T_S_viapoints(:,:,1) = T_S_init_final;

T_S_viapoints(:,:,2) = T_S_init_lift;

T_S_viapoints(:,:,3) = T_S_0*T_0_Cube*T_Cube_Z_1*T_surface_tool_tip;   

T_S_viapoints(:,:,4) = T_S_0*T_0_Cube*T_Cube_Z_2*T_surface_tool_tip;

T_S_viapoints(:,:,5) = T_S_0*T_0_Cube*T_Cube_Z_3*T_surface_tool_tip;

T_S_viapoints(:,:,6) = T_S_0*T_0_Cube*T_Cube_Z_4*T_surface_tool_tip;

T_S_viapoints(:,:,7) = T_S_0*T_0_Cube*T_Cube_Z_5*T_surface_tool_tip;

T_S_viapoints(:,:,8) = T_S_0*T_0_Cube*T_Cube_B_1*T_surface_tool_tip;

qrt_init_test = [0 0 0 0 -pi/2 0]; 

%% Joint Space Representation

desired_time_intervals = ...
    [ 4.00 ...        % Initial Configuration ---> Initial Lift
      4.00 ...        % Initial Lift ---> Point 1: 'Z'
      6.00 ...        % Point 1: 'Z' ---> Point 2: 'Z'
      6.00 ...        % Point 2: 'Z' ---> Point 3: 'Z'
      6.00 ...        % Point 3: 'Z' ---> Point 4: 'Z'
      4.00 ...        % Point 4: 'Z' ---> Lift Point Between Letter 1 & 2
      4.00];       
  
desired_accelerations = ...
    [10, ...           % theta1_accel [units: rad/(s^2)]
     12, ...           % theta2_accel [units: rad/(s^2)]
     12, ...           % theta3_accel [units: deg/(s^2)]
      8, ...           % d4_accel [units: meters/(s^2)]
      8, ...
      8];
  
% Convert from [rad/(s^2)] ---> [deg/(s^2)]
desired_accelerations = rad2deg(desired_accelerations);

[qrt, time] = ...
    generate_trajectory_linear_parabolic(puma, T_S_viapoints, T_0_S, ...
    desired_time_intervals, desired_accelerations, 0.00001, ...
    puma.tool, qrt_init_test);

% T_tool_test = T_0_Cube*T_Cube_Z_1*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_Z_2*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_Z_3*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_Z_4*T_surface_tool_tip
% 
% T_tool_test = T_0_Cube*T_Cube_B_1*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_2*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_3*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_4*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_5*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_6*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_7*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_8*T_surface_tool_tip
% T_tool_test = T_0_Cube*T_Cube_B_9*T_surface_tool_tip
% 
% T_tool_test = T_0_Cube*T_Cube_L_1*T_surface_tool_tip
% 
% T_tool_test = T_0_Cube*T_Cube_L_3*T_surface_tool_tip
% 
% qrt_ = ik_puma560(puma, T_tool_test, 'rad', qrt_init_test)

%%
sample_step_animation = 10000;

time_sample_animation = time(1:sample_step_animation:numel(time));

qrt_sample_animation = qrt(1:sample_step_animation:numel(time), :);     

qrt_sample_animation = deg2rad_q_trajectory(puma, qrt_sample_animation);

%%
clf
disp('Starting to Dispay')
disp(path_to_CAD)
ae = [200 30];
figure

plot_metal_cube(T_0_Cube)
hold on

figure_workspace = [-0.2 1.00 -0.2 0.8 -0.672 0.6];

puma_CAD_model.plot3d(qrt_sample_animation, 'view', ae, ...
    'path', path_to_CAD, ...
    'workspace', [-0.2 1.00 -0.2 0.8 -0.672 0.6]);

%% 
% Sampling for plotting purposes 1000 * 0.00001 = 0.01s == 100 Hz
sample_step = 1000;

time_sample = time(1:sample_step:numel(time));

qrt_sample = qrt(1:sample_step:numel(time), :);     

qrt_sample = deg2rad_q_trajectory(puma, qrt_sample);

time_end_phases_stamps = [ 
    sum(desired_time_intervals(1:2)); ...
    sum(desired_time_intervals(1:5)); ...
    sum(desired_time_intervals(1:7))];

plot_trajectory(puma, time_sample, qrt_sample, qrt_sample, ...
    time_end_phases_stamps, 'rad');
