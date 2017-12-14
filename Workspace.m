% Workspace of 5DoF Lynxmotion Arm
%----------------------------------
% Robotics Fundamentals Coursework
% Greg Baker

clear all
close all

%% Initialise joint angles

% Specify limiting angles of each joint
    load('stored_variables/limits.mat')
        
% Initialise joint angles       
    q = [0 0 0 0 0];
    
% Coordinates of arm zero position
    coords = Forward_Kinematics(q);

% Number of steps between limits
    n = 10;

% Initialise plot
    h = figure('visible','off');
    hold on
    grid on
    plot3(coords(1,:), coords(2,:), coords(3,:), 'r', 'Linewidth', 3)

%% Loop through range of motion for each joint angle
            
for i = 1:n
    f = (limits(4,2)-limits(4,1))/n;
    q(4) = limits(4,1) + (i-1)*f + rand(1)*f;
    
    for j = 1:n
        e = (limits(3,2)-limits(3,1))/n;
        q(3) = limits(3,1) + (j-1)*e + rand(1)*e;
        
        for k = 1:n
            d = (limits(2,2)-limits(2,1))/n;
            q(2) = limits(2,1) + (k-1)*d + rand(1)*d;

            for l = 1:n
                c = (limits(1,2)-limits(1,1))/n;
                q(1) = limits(1,1) + (l-1)*c + rand(1)*c;

                coords = Forward_Kinematics(q);

                x = coords(1,6);
                y = coords(2,6);
                z = coords(3,6);

                plot3(x,y,z,'b.');
                
            end
        end
    end
end

%% Plot

xlim([-400 400]);
ylim([-400 400]);
set(gca, 'FontSize', 14);
xlabel('\fontsize{16}x');
ylabel('\fontsize{16}y');
zlabel('\fontsize{16}z');
view(3)
hold off;
set(h,'visible','on');