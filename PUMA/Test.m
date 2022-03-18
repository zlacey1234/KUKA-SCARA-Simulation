clear all
close 
clc

g = 9.81;

px_c2 = 0.068;
py_c2 = 0.006;
pz_c2 = 0.2;

q2 = -pi/4;
q2dot = 0.1;
q2dotdot = 0.01;

q1dot = 0.1;
q1dotdot = 0.01;

A = g*cos(q2) + pz_c2*q1dotdot*sin(q2) - py_c2*q1dot^2*sin(q2)^2 + px_c2*q1dot^2*cos(q2)*sin(q2) - 2*px_c2*q1dot*q2dot*sin(q2)

B = g*cos(q2) + pz_c2*(q1dotdot*sin(q2) + q1dot*q2dot*cos(q2)) - q1dot*sin(q2)*(px_c2*(q2dot - q1dot*cos(q2)) + py_c2*q1dot*sin(q2))

C = g*cos(q2) + pz_c2*(q1dotdot*sin(q2) + q1dot*q2dot*cos(q2)) - q1dot*sin(q2)*(px_c2*(q2dot - q1dot*cos(q2)) + py_c2*q1dot*sin(q2))