%% M-File: PUMADynamicsTest
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

% Gravitational Acceleration [units: m/(s^2)]
g = sym('g', 'real');
    
% Number of Joints 
number_of_joints = puma_560.n();
    
% Joint (joint)
joints = sym('q%d', [1 number_of_joints], 'real');
    
% Joint Velocity (joint-dot)
dot_joint = sym('q%ddot', [1 number_of_joints], 'real');

dt1 = dot_joint(1)
dt2 = dot_joint(2)
dt3 = dot_joint(3)
dt4 = dot_joint(4)
dt5 = dot_joint(5)
dt6 = dot_joint(6)

% Joint Acceleration (joint-dotdot)
dotdot_joint = sym('q%ddotdot', [1 number_of_joints], 'real');

ddt1 = dotdot_joint(1)
ddt2 = dotdot_joint(2)
ddt3 = dotdot_joint(3)
ddt4 = dotdot_joint(4)
ddt5 = dotdot_joint(5)
ddt6 = dotdot_joint(6)

syms Ixx_c1 Iyy_c1 Izz_c1 Ixx_c2 Iyy_c2 Izz_c2 Ixx_c3 Iyy_c3 Izz_c3 real
IC1 = eye(3).*[Ixx_c1; Iyy_c1; Izz_c1]
IC2 = eye(3).*[Ixx_c2; Iyy_c2; Izz_c2]
IC3 = eye(3).*[Ixx_c3; Iyy_c3; Izz_c3]

%%
T_0_1 = puma_560.A([1], joints)
T_0_2 = simplify(puma_560.A([1, 2], joints))
T_0_3 = simplify(puma_560.A([1, 2, 3], joints))
% T_0_4 = simplify(puma_560.A([1, 2, 3, 4], joints))
% T_0_5 = simplify(puma_560.A([1, 2, 3, 4, 5], joints))
% T_0_6 = simplify(puma_560.A([1, 2, 3, 4, 5, 6], joints))

T_1_2 = puma_560.A([2], joints)
T_1_3 = simplify(puma_560.A([2, 3], joints))

T_2_3 = puma_560.A([3], joints)

T_3_4 = puma_560.A([4], joints)


%%
[R0_1, P0_1] = tr2rt(T_0_1); R1_0 = transpose(R0_1);
[R1_2, P1_2] = tr2rt(T_1_2); R2_1 = transpose(R1_2);
[R2_3, P2_3] = tr2rt(T_2_3) 
R3_2 = transpose(R2_3);
[R3_4, P3_4] = tr2rt(T_3_4); R4_3 = transpose(R3_4);

ft = zeros(3,1); 

nt = zeros(3,1); 

w0 = zeros(3,1);

wd0 = zeros(3,1); 

v0 = zeros(3,1); 

vd0 = [0 ; 0 ; g];


p1c1 = transpose(L(1).r)
p2c2 = transpose(L(2).r)
p3c3 = transpose(L(3).r)




%% Outward Iteration

% i = 0
w1 = R1_0*w0 + dt1*R0_1(1:3,3);
wd1 = R1_0*wd0 + R1_0*cross(w0,dt1*R0_1(1:3,3)) +ddt1*R0_1(1:3,3);
vd1 = R1_0*(cross(wd0,P0_1)+cross(w0,cross(w0,P0_1))+vd0);
vcd1 = cross(wd1,p1c1) + cross(w1,cross(w1,p1c1))+vd1

F1 = m1 * vcd1 
N1 = IC1 * wd1 + cross(w1,IC1*w1)

% i = 1
w2 = R2_1*w1 + dt2*R1_2(1:3,3)
wd2 = R2_1*wd1 + R2_1*cross(w1,dt2*R1_2(1:3,3)) +ddt2*R1_2(1:3,3)
vd2 = R2_1*(cross(wd1,P1_2)+cross(w1,cross(w1,P1_2))+vd1);
vcd2 = cross(wd2,p2c2) + cross(w2,cross(w2,p2c2))+vd2

F2 = m2 * vcd2 
N2 = IC2 * wd2 + cross(w2,IC2*w2)

% i = 2
w3 = R3_2*w2 + dt3*R2_3(1:3,3)
wd3 = R3_2*wd2 + R3_2*cross(w2,dt3*R2_3(1:3,3)) +ddt3*R2_3(1:3,3)
vd3 = R3_2*(cross(wd2,P2_3)+cross(w2,cross(w2,P2_3))+vd2);
vcd3 = cross(wd3,p3c3) + cross(w3,cross(w3,p3c3))+vd3

F3 = m3 * vcd3 
N3 = IC3 * wd3 + cross(w3,IC3*w3)

%% Inward Iteration
syms fx fy fz nx ny nz real

f4 = [fx fy fz]'; 
n4 = [nx ny nz]'; 

% i = 3
f3 = R3_4 * f4 + F3
n3 = N3 + R3_4*n4 + cross(p3c3, F3) + cross(P3_4, R3_4 * f4)

R2_3(1:3,3)

% (revolute)
tau3 = n3' * R2_3(1:3,3)


% i = 2
f2 = R2_3 * f3 + F2
n2 = N2 + R2_3*n3 + cross(p2c2, F2) + cross(P2_3, R2_3 * f3)

R1_2(1:3,3)

% (revolute)
tau2 = n2' * R1_2(1:3,3)


% i = 1
f1 = R1_2 * f2 + F1
n1 = N1 + R1_2*n2 + cross(p1c1, F1) + cross(P1_2, R1_2 * f2)

R0_1(1:3,3)

% (revolute)
tau1 = n1' * R0_1(1:3,3)

TAU = [tau1; tau2; tau3]

simplify(TAU)