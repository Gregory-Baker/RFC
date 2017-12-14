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
    figure(1)
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
    for i = 1:n_points
        plot3(poe(1,i), poe(2,i), poe(3,i), 'ko');
        text(poe(1,i), poe(2,i)+15, poe(3,i), num2str(i));
    end
    
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
    plot3(poe_free(1,:),poe_free(2,:), poe_free(3,:), 'b');
    if animation_on
        pause(0.2)
    end
    set(h3,'Visible','off')
end
% Finalise plot
    plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
    hold off


%% Free motion trajectory with parabolic blends


% Define global acceleration (deg/sec^2)
    accn_mag = 30;
        
% Returns 
for i = 1:n_joints
    
    joint_positions(1,:) = joint_angles_all(:,i);
    
    joint_values{i} = parabolic_blend( joint_positions, accn_mag, T);
    
end

% Index value
    k = 1;

% Initialise animated figure for free motion
    figure(2)
    hold on;
    grid on;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-200 200])
    zlim([-100  250])
    view([100 30])
    set(gca, 'FontSize', 14);
    for i = 1:n_points
        plot3(poe(1,i), poe(2,i), poe(3,i), 'ko');
        text(poe(1,i), poe(2,i)+15, poe(3,i), num2str(i));
    end
   
for i = 1:200
    
    n_samples = size(joint_values{1},2);
    
    q_free(i,:) = [joint_values{1}(2,k), joint_values{2}(2,k), joint_values{3}(2,k), joint_values{4}(2,k), 0];
    
    coords = Forward_Kinematics(q_free(i,:));
    
    for l = 1:3
        poe_para(l,i) = coords(l,6);
    end
 
    
% Plot animation at one quarter of sample rate
    if mod(i,4) == 0
        if exist('h2')
            set(h2,'Visible','off')
        end
        h2 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
        plot3(poe_para(1,:),poe_para(2,:), poe_para(3,:), 'r');
    end
    
    if animation_on
        pause(0.04)
    end
    
    k = k + (n_samples-1)/200;
end
% Finalise plot
    hold off
    pause(1)

%% Angle, Velocity and Acceleration Plots (Parabolic Blends)

% Joint angle vs time plot for free motion
    figure(3)
    hold on
    for i = 1:n_joints
        plot(joint_values{i}(1,:), joint_values{i}(2,:));
    end
    legend('\theta_1', '\theta_2', '\theta_3', '\theta_4','Orientation','horizontal', 'Location','northoutside');
    xlabel('time (sec)')
    ylabel('$\theta$ (deg)', 'Interpreter','latex')
    hold off
    set(gca, 'FontSize', 14);
    fig = gcf;
    fig.PaperUnits = 'inches';
    fig.PaperPosition = [0 0 6 4];
    print('Figures/joint-angles-plot','-depsc','-r0')
    if animation_on
        pause(1)
    end

% Joint velocity vs time plot for free motion
    figure(4)
    hold on
    for i = 1:n_joints
        plot(joint_values{i}(1,:), joint_values{i}(3,:));
    end
    %legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
    xlabel('time (sec)')
    ylabel('$\dot{\theta}$ (deg/sec)', 'Interpreter','latex')
    hold off
    set(gca, 'FontSize', 14);
    fig = gcf;
    fig.PaperUnits = 'inches';
    fig.PaperPosition = [0 0 6 4];
    print('Figures/joint-velocities-plot','-depsc','-r0')
    if animation_on
        pause(1)
    end

% Joint acceleration vs time plot for free motion
    figure(5)
    hold on
    for i = 1:n_joints
        plot(joint_values{i}(1,:), joint_values{i}(4,:));
    end
    %legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
    xlabel('time (sec)')
    ylabel('$\ddot{\theta}$ $(deg/sec^2)$', 'Interpreter','latex')
    set(gca, 'FontSize', 14);
    hold off
    fig = gcf;
    fig.PaperUnits = 'inches';
    fig.PaperPosition = [0 0 6 4];
    print('Figures/joint-accelerations-plot','-depsc','-r0')
    if animation_on
        pause(1)
    end

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
    view([100 30])
    set(gca, 'FontSize', 14);
    for i = 1:n_points
        plot3(poe(1,i), poe(2,i), poe(3,i), 'ko');
        text(poe(1,i), poe(2,i)+15, poe(3,i), num2str(i));
    end
    
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
    h4 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
    plot3(poe_lin(1,:),poe_lin(2,:), poe_lin(3,:), 'k');
    if animation_on
        pause(0.2)
    end
    set(h4,'Visible','off')
end
plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
hold off

%% Trajectory Comparison Plot

% Plot arm positions at each of the five points
    figure(7)
    grid on
    hold on
    for i = 1:n_points
        plot3(poe(1,i), poe(2,i), poe(3,i), 'k*', 'MarkerSize', 12);
        text(poe(1,i), poe(2,i)+15, poe(3,i), num2str(i));
    end
    l1 = plot3(poe_free(1,:), poe_free(2,:), poe_free(3,:), 'b');
    l2 = plot3(poe_para(1,:), poe_para(2,:), poe_para(3,:), 'r');
    l3 = plot3(poe(1,:), poe(2,:), poe(3,:), 'k');
    legend([l1 l2 l3],{'Free Motion', 'Via Points', 'Linear Trajectory'}, 'Location','southwest');
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold off
    set(gca, 'FontSize', 14);
    if animation_on
        pause(1)
    end
    
    
%% Obstacle avoidance + animation

% Object after specified point 
    obj_aft = 4;

% Location and radius of spherical object (mm)
    h = (p(obj_aft,1)+p(obj_aft+1,1))/2;
    j = (p(obj_aft,2)+p(obj_aft+1,2))/2;  
    k = (p(obj_aft,3)+p(obj_aft+1,3))/2;
    r = 25;
    
% Number of points between start point and target
    n = 100;
    
% Index variables
    m = 1;

% Coordinate system for spherical obstacles   
    [xs,ys,zs] = sphere;
    
% Initialise figure with start and end point and spherical obstacles
    figure(8)
    grid on
    hold on
    view([100 30])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-200 200])
    zlim([-100  250])
    set(gca, 'FontSize', 14);
    for i = 1:n_points
        plot3(poe(1,i), poe(2,i), poe(3,i), 'ko');
        text(poe(1,i), poe(2,i)+15, poe(3,i), num2str(i));
    end
    
% Plot spherical obstacles
    surf(xs*r+h, ys*r+j, zs*r+k)

% Loop through all points in the task
for i = 1:(n_points-1)
    
    % Index variable
        a = 1;
        
    % Initialise intermediate point arrays
        x_int = zeros(1,n);
        y_int = zeros(1,n);
        z_int = zeros(1,n);
        pitch_int = zeros(1,n);
    
    % First point in array is previous target point
        x_int(a) = p(i,1);
        y_int(a) = p(i,2);
        z_int(a) = p(i,3);
        pitch_int(a) = p(i,4);
        
    % Loop until at target point
    while round(x_int(a)) ~= p(i+1,1) || round(y_int(a)) ~= p(i+1,2) || round(z_int(a) ~= p(i+1,3)) || round(pitch_int(a) ~= p(i+1,4))         
        
    % Find steps between current point and next target point
        x_int(:) = linspace(x_int(a), p(i+1,1), n);
        y_int(:) = linspace(y_int(a), p(i+1,2), n);
        z_int(:) = linspace(z_int(a), p(i+1,3), n);
        pitch_int(:) = linspace(pitch_int(a), p(i+1, 4), n);
        
        % Iterate through intermediate points
        for a = 1:n
            
        % Check if point is within 10mm of spherical object
            if (x_int(a)-h)^2 + (y_int(a)-j)^2 + (z_int(a)-k)^2 < (r+10)^2
            
            % Offset x by 5 and recalculate intermediate steps to target
                x_int(a) = x_int(a) - 5;
                break
            end
            
            joint_angles = inverse_kinematics(x_int(a),y_int(a),z_int(a),pitch_int(a));

            coords = Forward_Kinematics(joint_angles);

            for l = 1:3
                poe_av(l,m) = coords(l,6);
            end

            if exist('h5')
                set(h5,'Visible','off')
            end
            h5 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
            
            if animation_on
                pause(0.02)
            end
            
            plot3(poe_av(1,:), poe_av(2,:), poe_av(3,:), 'r', 'LineWidth', 2)
            m = m+1;
        end   
    end 
end




