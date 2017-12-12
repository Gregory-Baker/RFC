% Call for Inverse Kinemtatics of 5DoF arm
%----------------------------------
% Robotics Fundamentals Coursework
% Greg Baker

clear all
close all

%% Initialise

% Load joint limits
    load('stored_variables/limits.mat');
    
% Pick random values in the range of each joint
    for i = 1:5
        q(i) = limits(i,1)+rand(1)*(limits(i,2)-limits(i,1));
    end

    display(q)
    
% Display pitch
    pitch = q(2)+q(3)+q(4)

%% Forward kinematics

% Coordinates of end points of each link
    coords = Forward_Kinematics(q)

%figure('Forward Kinematics');
    plot_arm(coords);
    

%% Inverse Kinematics

% End effector coordinates for IK
    x = coords(1,6);
    y = coords(2,6);
    z = coords(3,6);
    
    pitch = q(2)+q(3)+q(4);

% Determine joint angles from Inverse Kinematics
    joint_angles = inverse_kinematics(x, y, z, pitch);

% Calculate coordinates of each link
    coords = Forward_Kinematics(joint_angles);

% Plot arm
    plot_arm(coords);



