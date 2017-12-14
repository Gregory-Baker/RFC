%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacle avoidance script
% Greg Baker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc

%% Location of waypoints and obstacles

% location of Start and End points [x y z] (mm)
    p(1,:) = [150, -100, 200];
    p(2,:) = [150, 100, 200];

% Location and radius of spherical objects (mm)
    h = 150;
    j = 0;   % Second sphere at j = +50 (mm)
    k = 200
    r = 40;

%% Straight line to target

% Number of points between start point and target
    n = 20;

% Pitch kept constant and zero throughout
    pitch = 0;

% Initialise matrices for intermediate points
    x = zeros(1,n);
    y = zeros(1,n);
    z = zeros(1,n);

% Index variables
    i = 1;
    m = 1;

% Set first point to starting point
    x(i) = p(1,1);
    y(i) = p(1,2);
    z(i) = p(1,3);

% Coordinate system for spherical obstacles   
    [xs,ys,zs] = sphere;

% Initialise figure with start and end point and spherical obstacles
    figure(1)
    grid on
    hold on
    view([100 30])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-150 150])
    zlim([0  250])
    
% Plot spherical obstacles
    surf(xs*r+h, ys*r+j, zs*r+k)
    surf(xs*r+h, ys*r-j, zs*r+k)
    
% Show location of start point in 3D plot
    plot3(p(2,1), p(2,2), p(2,3), 'k*', 'MarkerSize', 12)
    text(p(2,1), p(2,2)+10, p(2,3), 'Target', 'FontSize', 12)
    
% Show location of start point in 3D plot
    plot3(p(1,1), p(1,2), p(1,3), 'k*', 'MarkerSize', 12)
    text(p(1,1), p(1,2)-10, p(1,3), 'Start', 'FontSize', 12, 'HorizontalALignment', 'right')

while round(x(i)) ~= p(2,1) || round(y(i)) ~= p(2,2) || round(z(i) ~= p(2,3))
    % Matrix of sampling points
        x(:) = linspace(x(i), p(2,1), n);
        y(:) = linspace(y(i), p(2,2), n);
        z(:) = linspace(z(i), p(2,3), n);

    for i = 1:n

        if (x(i)-h)^2 + (y(i)-j)^2 + (z(i)-k)^2 < (r+5)^2 || (x(i)-h)^2 + (y(i)+j)^2 + (z(i)-k)^2 < (r+5)^2
            random = rand(1);
            if random <= (1/2)
                z(i) = z(i) + 5;
            else
                x(i) = x(i) - 5;
            end
            break
        end

        joint_angles = inverse_kinematics(x(i),y(i),z(i),pitch);

        coords = Forward_Kinematics(joint_angles);
        
        for l = 1:3
            poe(l,m) = coords(l,6);
        end
        
        if exist('h2')
            set(h2,'Visible','off')
        end
        h2 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
        pause(0.3)
        plot3(poe(1,:), poe(2,:), poe(3,:), 'r--', 'LineWidth', 2)
        m = m+1;
    end
    
end

    