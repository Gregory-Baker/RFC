% Call for Inverse Kinemtatics of 5DoF arm
%----------------------------------
% Robotics Fundamentals Coursework
% Greg Baker

clear all
close all

%% Load variables

% Load joint limits
    load('stored_variables/limits.mat');
    
    
%% Input desired position (mm) and pitch (deg) of end effector

x = 200;
y = 0;
z = 200;

pitch = -30;

%% Call function and plot

joint_angles = inverse_kinematics(x, y, z, pitch);

