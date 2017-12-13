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
    animation_on = false;
    

%% Coordinates of 5 positions

% 
for i = 1:n_points
    
% Joint angles from IK
    joint_angles = inverse_kinematics (p(i,1),p(i,2),p(i,3),p(i,4));
    joint_angles_all(i,:) = joint_angles;

% Coordinates of links from FK
    point_coords{i} = Forward_Kinematics(joint_angles);

% Point of end effector
    poe(:,i) = point_coords{i}(:,6);

end
    
    
%% Free motion trajectory with parabolic blends


% Define global acceleration (deg/sec^2)
    accn_mag = 30;
        
% Returns 
for i = 1:n_joints
    
    joint_positions(1,:) = joint_angles_all(:,i);
    
    joint_values{i} = parabolic_blend( joint_positions, accn_mag, T);
    
end

k = 1;
for i = 1:200
    
    n_samples = size(joint_values{1},2);
    
    q_free(i,:) = [joint_values{1}(2,k), joint_values{2}(2,k), joint_values{3}(2,k), joint_values{4}(2,k), 0];
    
    coords = Forward_Kinematics(q_free(i,:));
    
    for l = 1:3
        poe_free(l,i) = coords(l,6);
    end
    
% Initialise animated figure for free motion
    figure(1)
    hold on;
    grid on;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-200 200])
    zlim([-100  250])
    view([80 30])
    
% Plot animation at one quarter of sample rate
    if mod(i,4) == 0
        if exist('h2')
            set(h2,'Visible','off')
        end
        h2 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
        plot3(poe_free(1,:),poe_free(2,:), poe_free(3,:), 'r--');
    end
    
    k = k + (n_samples-1)/200;
end
plot3(poe(1,:), poe(2,:), poe(3,:), 'go');
hold off
pause(1)

%% Joint angle, velocity and acceleration plots for free motion + parabolic blends

figure(2)
grid on
hold on
for i = 1:n_points
    plot3(point_coords{i}(1,:), point_coords{i}(2,:), point_coords{i}(3,:), 'LineWidth', 2)
end
plot3(poe_free(1,:), poe_free(2,:), poe_free(3,:), 'r--');
plot3(poe(1,:), poe(2,:), poe(3,:), 'k--');
view([80 30])
xlabel('x')
ylabel('y')
zlabel('z')
hold off
pause(1)

figure(3)
hold on
for i = 1:n_joints
plot(joint_values{i}(1,:), joint_values{i}(2,:));
end
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
hold off
pause(1)

figure(4)
hold on
for i = 1:n_joints
plot(joint_values{i}(1,:), joint_values{i}(3,:));
end
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
xlabel('time (seconds)')
ylabel('$\ddot{\theta}$', 'Interpreter','latex')
hold off
pause(1)

figure(5)
hold on
for i = 1:n_joints
plot(joint_values{i}(1,:), joint_values{i}(4,:));
end
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
hold off


%% Straight Line Trajectory

% Matrix of sampling points
for i = 1:(n_points-1)
    
    x(i,:) = linspace(p(i,1), p(i+1,1), n);
    y(i,:) = linspace(p(i,2), p(i+1,2), n);
    z(i,:) = linspace(p(i,3), p(i+1,3), n);
    pitch(i,:) = linspace(p(i,4), p(i+1,4), n);

end

% Reshape matrices into arrays containing all points
m = 1;
for j = 1:(n_points-1)
    for i = 1:(n-1)
        x2(m) = x(j,i);
        y2(m) = y(j,i);
        z2(m) = z(j,i);
        pitch2(m) = pitch(j,i);
        m = m + 1;
    end
end

% Add final point onto arrays
    x2(m) = x((n_points-1),n);
    y2(m) = y((n_points-1),n);
    z2(m) = z((n_points-1),n);
    pitch2(m) = pitch((n_points-1),n);
   
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
    view([80 30])
    
% Calculate joint angles for all sampled points using IK 
for i = 1:(n_points-1)*(n-1)+1

    joint_angles = inverse_kinematics(x2(i), y2(i), z2(i), pitch2(i));
    
% Coordinates of links from FK
    coords = Forward_Kinematics(joint_angles);
    
% Point of end effector for linear trajectory
    for l = 1:3
        poe_lin(l,i) = coords(l,6);
    end
        

% Plot arm trajectory
    if exist('h4')
        set(h4,'Visible','off')
    end
    h4 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
    plot3(poe_lin(1,:),poe_lin(2,:), poe_lin(3,:), 'k--');
    if animation_on
        pause(0.2)
    end
end
plot3(poe(1,:), poe(2,:), poe(3,:), 'go');
hold off





