% Plan A Task

clear all
close all
clc

%% Input Coordinates

% p = [x_pos, y_pos, z_pos, pitch]

    p(1,:) = [6, 46, 70, -136];
    p(2,:) = [129, 150, 150, 30];
    p(3,:) = [150, 80, 70, -20];
    p(4,:) = [136, -69, -67, -78];
    p(5,:) = [60, -146, 170, 36];
    
% Total time for task
    T = 20;

% Number of points
    n_points = size(p,1);

% Number of active joints
    n_joints = size(p,2);
    
% Time interval between waypoints
    time_int = T/(n_points-1);
    
% Number of samples between each point
    n = 10;
    
% Animation on
    animation_on = true;
    

%% Coordinates of 5 positions

for i = 1:n_points
    
% Joint angles from IK
    joint_angles = inverse_kinematics (p(i,1),p(i,2),p(i,3),p(i,4));
    joint_angles_all(i,:) = joint_angles;

% Coordinates of links from FK
    point_coords{i} = Forward_Kinematics(joint_angles);

% Point of end effector
    poe(:,i) = point_coords{i}(:,6);

end
    
    
%% Free motion trajectory

% Matrix of sampling points
for i = 1:(n_points-1)
    
    q1(i,:) = linspace(joint_angles_all(i,1), joint_angles_all(i+1,1), n);
    q2(i,:) = linspace(joint_angles_all(i,2), joint_angles_all(i+1,2), n);
    q3(i,:) = linspace(joint_angles_all(i,3), joint_angles_all(i+1,3), n);
    q4(i,:) = linspace(joint_angles_all(i,4), joint_angles_all(i+1,4), n);

end

% Reshape matrices into arrays containing all points
m = 1;
for j = 1:(n_points-1)
    for i = 1:(n-1)
        q1_2(m) = q1(j,i);
        q2_2(m) = q2(j,i);
        q3_2(m) = q3(j,i);
        q4_2(m) = q4(j,i);
        m = m + 1;
    end
end

% Add final point onto arrays
    q1_2(m) = q1((n_points-1),n);
    q2_2(m) = q2((n_points-1),n);
    q3_2(m) = q3((n_points-1),n);
    q4_2(m) = q4((n_points-1),n);

% Linear trajectory animation figure initialisation
    figure(6)
    grid on
    hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-200 200])
    zlim([-100  250])
    view([100 30])
    set(gca, 'FontSize', 14);
    
% Calculate joint angles for all sampled points using IK 
for i = 1:(n_points-1)*(n-1)+1

% Joint angle array
    q = [q1_2(i), q2_2(i), q3_2(i), q4_2(i), 0];
    
% Coordinates of links from FK
    coords = Forward_Kinematics(q);
    
% Point of end effector for free motion trajectory
    for l = 1:3
        poe_free(l,i) = coords(l,6);
    end
        

% Plot arm trajectory
    h3 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
    plot3(poe_free(1,:),poe_free(2,:), poe_free(3,:), 'b--');
    if animation_on
        pause(0.2)
    end
    set(h3,'Visible','off')
end
plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
plot3(poe(1,:), poe(2,:), poe(3,:), 'go');
hold off




