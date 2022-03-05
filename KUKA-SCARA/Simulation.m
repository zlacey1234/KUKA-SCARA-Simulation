%% Simulation 
% Simulation for KUKA KR 6 R700 Z200 Robotic Arm

% Clear the Workspace
clear 
close all 
clc

path_to_workspace = updatePaths();

path_to_CAD = [path_to_workspace, '\KUKA_CAD'];

% Load the SCARA Model
mdl_scara_kuka_kr_6_r700z200

% Transformation to Stationary Frame {S} w.r.t the Base Frame {0}
T_0_S = [1  0  0  0.425;...
         0  1  0  0.275;...
         0  0  1  0;...
         0  0  0  1];

% Transformation to Center of the PCB {PCB} w.r.t the Stationary Frame {S}
T_S_PCB = [1  0  0  0;...
           0  1  0  0;...
           0  0  1  0.010;...
           0  0  0  1];

%% Homogeneous Transformations of the PCB and Feeder

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHIP #1                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation to Chip #1 w.r.t the Center of the PCB {PCB}
T_PCB_Chip1 = [ 0 -1  0   0.045;...
                1  0  0  -0.045;...
                0  0  1   0;...
                0  0  0   1];
       
% Transformation to Chip #1 w.r.t the Stationary Frame {S}
T_S_Chip1 = T_S_PCB*T_PCB_Chip1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHIP #2                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation to Chip #2 w.r.t the Center of the PCB {PCB}
T_PCB_Chip2 = [ 1  0  0  -0.045;...
                0  1  0  -0.045;...
                0  0  1   0;...
                0  0  0   1];
       
% Transformation to Chip #2 w.r.t the Stationary Frame {S}
T_S_Chip2 = T_S_PCB*T_PCB_Chip2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHIP #3                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation to Chip #3 w.r.t the Center of the PCB {PCB}
T_PCB_Chip3 = [ 0  1  0  -0.045;...
               -1  0  0   0.045;...
                0  0  1   0;...
                0  0  0   1];
       
% Transformation to Chip #3 w.r.t the Stationary Frame {S}
T_S_Chip3 = T_S_PCB*T_PCB_Chip3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHIP #4                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation to Chip #4 w.r.t the Center of the PCB {PCB}
T_PCB_Chip4 = [-1  0  0   0.045;...
                0 -1  0   0.045;...
                0  0  1   0;...
                0  0  0   1];
       
% Transformation to Chip #4 w.r.t the Stationary Frame {S}
T_S_Chip4 = T_S_PCB*T_PCB_Chip4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FEEDER                                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_PCB_feeder = 0.1;
y_PCB_feeder = 0.0;
% Transformation to Feeder w.r.t the Center of the PCB {PCB}
T_PCB_feeder = [ 1  0  0  x_PCB_feeder;...
                 0  1  0  y_PCB_feeder;...
                 0  0  1  0;...
                 0  0  0  1];
             
% Transformation to Feeder w.r.t the Stationary Frame {S}
T_S_feeder = T_S_PCB*T_PCB_feeder;

% Transformation to Feeder w.r.t the Base Frame {0}
T_0_feeder = T_0_S*T_S_feeder;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial/Final SCARA Configuration                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transformation to Initial/Final Configuration w.r.t the Stationary 
% Frame {S}
T_S_init_final = [0 -1  0  0;...
                  1  0  0  0;...
                  0  0  1  0.080;...
                  0  0  0  1];


              
%% Via Point Representation 

lift_height = 0.01;

T_S_viapoints = zeros(4, 4, 1);

T_add_forward_viapoint = [0  0  0  0.010;...
                          0  0  0  0.000;...
                          0  0  0  0.005;...
                          0  0  0  0];
                      
T_add_height_viapoint = [0  0  0  0.000;...
                         0  0  0  0.000;...
                         0  0  0  0.002;...
                         0  0  0  0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First Via Point (Initial Arm Configuration) w.r.t the Stationary       %
% Frame {S}                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,1) = T_S_init_final; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Second Via Point (At Feeder) w.r.t the Stationary Frame {S}            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,2) = T_S_feeder + T_S_feeder*T_add_height_viapoint;;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Third Via Point (At Feeder closing the gripper) w.r.t the Stationary   %
% Frame {S}                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,3) = T_S_feeder + T_S_feeder*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fourth Via Point (Midpoint between Feeder and Chip #1 at 'lift_height')%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fourth Via Point w.r.t the Center of the PCB {PCB}
T_PCB_v4 = [cos(pi/4)  -sin(pi/4)  0  (x_PCB_feeder + 0.045)/2; ...
            sin(pi/4)   cos(pi/4)  0  (y_PCB_feeder + -0.045)/2; ...
            0           0          1  lift_height; ...
            0           0          0  1];
        
% Fourth Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,4) = T_S_PCB*T_PCB_v4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fifth Via Point (At Chip #1)  w.r.t the Stationary Frame {S}           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,5) = T_S_Chip1 + T_S_Chip1*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sixth Via Point (At Chip #1 opening the gripper) w.r.t the Stationary  %
% Frame {S}                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,6) = T_S_Chip1 + T_S_Chip1*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Seventh Via Point (Move Safely Away from Chip #1 at '0.005' meters in  %
% height)                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Seventh Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,7) = T_S_Chip1 + T_S_Chip1*T_add_forward_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eight Via Point (Midpoint between Feeder and Chip #1 at 'lift_height') %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eight Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,8) = T_S_viapoints(:,:,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nineth Via Point (At Feeder) w.r.t the Stationary Frame {S}            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,9) = T_S_feeder + T_S_feeder*T_add_height_viapoint;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tenth Via Point (At Feeder closing the gripper) w.r.t the Stationary   %
% Frame {S}                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,10) = T_S_feeder + T_S_feeder*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eleventh Via Point (Midpoint between Feeder and Chip #2 at             %
% 'lift_height')                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eleventh Via Point w.r.t the Center of the PCB {PCB}
T_PCB_v11 = [1   0   0  (x_PCB_feeder + -0.045)/2; ...
             0   1   0  (y_PCB_feeder + -0.045)/2; ...
             0   0   1  lift_height; ...
             0   0   0  1];
        
% Eleventh Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,11) = T_S_PCB*T_PCB_v11;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twelvth Via Point (At Chip #2)  w.r.t the Stationary Frame {S}         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,12) = T_S_Chip2 + T_S_Chip2*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thirteenth Via Point (At Chip #2 opening the gripper) w.r.t the        %
% Stationary Frame {S}                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,13) = T_S_Chip2 + T_S_Chip2*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fourteenth Via Point (Move Safely Away from Chip #2 at '0.005' meters  %
% in height)                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,14) = T_S_Chip2 + T_S_Chip2*T_add_forward_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fifteenth Via Point (Midpoint between Feeder and Chip #2 at            %
% 'lift_height')                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fifteenth Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,15) = T_S_viapoints(:,:,11);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sixteenth Via Point (At Feeder) w.r.t the Stationary Frame {S}         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,16) = T_S_feeder + T_S_feeder*T_add_height_viapoint;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Seventeenth Via Point (At Feeder closing the gripper) w.r.t the        %
% Stationary Frame {S}                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,17) = T_S_feeder + T_S_feeder*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eighteenth Via Point (Midpoint between Feeder and Chip #3 at           %
% 'lift_height')                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Eighteenth Via Point w.r.t the Center of the PCB {PCB}
T_PCB_v18 = [cos(-pi/4)  -sin(-pi/4)  0  (x_PCB_feeder + -0.045)/2; ...
             sin(-pi/4)   cos(-pi/4)  0  (y_PCB_feeder + 0.045)/2; ...
             0            0           1  lift_height; ...
             0            0           0  1];
        
% Eighteenth Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,18) = T_S_PCB*T_PCB_v18;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nineteenth Via Point (At Chip #3)  w.r.t the Stationary Frame {S}      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,19) = T_S_Chip3 + T_S_Chip3*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenteeth Via Point (At Chip #3 opening the gripper) w.r.t the         %
% Stationary Frame {S}                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,20) = T_S_Chip3 + T_S_Chip3*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-first Via Point (Move Safely Away from Chip #3 at '0.005'       %
% meters in height)                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,21) = T_S_Chip3 + T_S_Chip3*T_add_forward_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-second Via Point (Midpoint between Feeder and Chip #3 at        %
% 'lift_height')                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-second Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,22) = T_S_viapoints(:,:,18);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-third Via Point (At Feeder) w.r.t the Stationary Frame {S}      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,23) = T_S_feeder + T_S_feeder*T_add_height_viapoint;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-fourth Via Point (At Feeder closing the gripper) w.r.t the      %
% Stationary Frame {S}                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,24) = T_S_feeder + T_S_feeder*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-fifth Via Point (Midpoint between Feeder and Chip #4 at         %
% 'lift_height')                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-fifth Via Point w.r.t the Center of the PCB {PCB}
T_PCB_v25 = [ 0  -1   0  (x_PCB_feeder + 0.045)/2; ...
              1   0   0  (y_PCB_feeder + 0.045)/2; ...
              0   0   1  lift_height; ...
              0   0   0  1];
        
% Twenty-fifth Via Point w.r.t the Stationary Frame {S}
T_S_viapoints(:,:,25) = T_S_PCB*T_PCB_v25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-sixth Via Point (At Chip #4)  w.r.t the Stationary Frame {S}    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,26) = T_S_Chip4 + T_S_Chip4*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-seventh Via Point (At Chip #4 opening the gripper) w.r.t the    %
% Stationary Frame {S}                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,27) = T_S_Chip4 + T_S_Chip4*T_add_height_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-eighth Via Point (Move Safely Away from Chip #3 at '0.005'      %
% meters in height)                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,28) = T_S_Chip4 + T_S_Chip4*T_add_forward_viapoint;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Twenty-nineth Via Point (Final Arm Configuration) w.r.t the Stationary %
% Frame {S}                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_S_viapoints(:,:,29) = T_S_init_final;

%% Joint Space Representation

desired_time_intervals = ...
    [2.25 ...         % (t_d{1_2})    Initial ---> Feeder 
     1.00 ...         % (t_d{2_3})    Closing Gripper
     0.87 ...         % (t_d{3_4})    Feeder ---> Lift Pose          
     0.87 ...         % (t_d{4_5})    Lift Pose ---> Chip #1
     0.60 ...         % (t_d{5_6})    Opening Gripper
     0.50 ...
     0.87 ...         % (t_d{7_8})    Chip #1 ---> Lift Pose          
     0.87 ...         % (t_d{8_9})    Lift Pose ---> Feeder
     1.00 ...         % (t_d{9_10})   Closing Gripper
     0.70 ...         % (t_d{10_11})  Feeder ---> Lift Pose          
     0.70 ...         % (t_d{11_12})  Lift Pose ---> Chip #2 
     0.50 ...         % (t_d{12_13})  Opening Gripper
     0.50 ...
     0.70 ...         % (t_d{14_15})  Chip #2 ---> Lift Pose      
     0.70 ...         % (t_d{15_16})  Lift Pose ---> Feeder
     1.10 ...         % (t_d{16_17})  Closing Gripper
     1.14 ...         % (t_d{17_18})  Feeder ---> Lift Pose
     1.14 ...         % (t_d{18_19})  Lift Pose ---> Chip #3  
     0.70 ...         % (t_d{19_20})  Opening Gripper
     0.60 ...
     1.14 ...         % (t_d{21_22})  Chip #3 ---> Lift Pose 
     1.14 ...         % (t_d{22_23})  Lift Pose ---> Feeder
     1.45 ...         % (t_d{23_24})  Closing Gripper
     1.30 ...         % (t_d{24_25})  Feeder ---> Lift Pose
     1.30 ...         % (t_d{25_26})  Lift Pose ---> Chip #4 
     0.85 ...         % (t_d{26_27})  Opening Gripper
     0.50 ...
     2.82];           % (t_d{28_29})  Chip #4 ---> Final          

desired_accelerations = ...
    [50, ...           % theta1_accel [units: deg/(s^2)]
     50, ...           % theta2_accel [units: deg/(s^2)]
     50, ...           % theta3_accel [units: deg/(s^2)]
     50, ...           % d4_accel [units: meters/(s^2)]
      0];
  
[qrt, time] = ...
    generate_trajectory_linear_parabolic(scara, T_S_viapoints, T_0_S, ...
    desired_time_intervals, desired_accelerations, 0.00001, T_wrist_tool);
%%
sample_step = 10000;

time_sample = time(1:sample_step:numel(time));

qrt_sample = qrt(1:sample_step:numel(time), :);              
              
              
% Joint Space of Initial SCARA Pose
T_0_init_final = T_0_S*T_S_init_final;
qrt_init = ik_scara_kuka_kr_6_r700z200(scara, ...
    T_0_init_final, T_wrist_tool, ...
    'elbowOption','up');

qrt_sample = deg2rad_q_trajectory(scara, qrt_sample);
qrt_init = deg2rad_q_trajectory(scara, qrt_init);

%%
clf
disp('Starting to Dispay')
disp(path_to_CAD)
ae = [200 30];
figure

figure_workspace = [-0.2 0.60 -0.2 0.4 0 0.6];

% Zoomed in View
%figure_workspace = [0.35 0.55 0.25 0.35 0 0.15];

plot_PCB(T_0_S, T_S_PCB, T_S_Chip1, T_S_Chip2, T_S_Chip3, T_S_Chip4, 1)
xlim(figure_workspace(1:2))
zlim(figure_workspace(3:4))
zlim(figure_workspace(5:6))
hold on
plot_feeder(T_0_feeder)

state_sample_trajectory = ...
    plot_via_points_and_trajectory_joint_space(scara, T_S_viapoints, ...
    T_0_S, qrt_sample, T_wrist_tool);

% plot_state_trajectory(state_sample_trajectory, time_sample, dt_sample);
title('KUKA KR 6 R700 Z200 (SCARA RRRP) Pick-and-Place')
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
%%
scara.plot3d(qrt_sample, 'view', ae, ...
    'path', path_to_CAD, ...
    'workspace', [-0.2 0.55 -0.2 0.4 0 0.6]);

