close all; clear all; clc
desired_state0 = desiredtrajectory_sim(0);
des_quad10 = [desired_state0(1:6)',1,0,0,0,0,0,0];
des_quad20 = [desired_state0(10:15)',1,0,0,0,0,0,0];
des_quad30 = [desired_state0(19:24)',1,0,0,0,0,0,0];


global params
%% ************************* Quad params********
params.mass = 0.030;  
params.grav = 9.81;  
params.I = [1.43e-5, 0, 0; 0, 1.43e-5, 0; 0, 0, 2.89e-5];
params.arm_length = 0.046; 
params.invI = inv(params.I);
params.maxangle = 40*pi/180;
params.maxF     = 2.5*params.mass*params.grav;  
params.minF     = 0.05*params.mass*params.grav; 