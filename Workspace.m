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

% Number of steps between limits
    n = 5;

% Initialise plot
    h = figure('visible','off');
    hold on;

%% Loop through range of motion for each joint angle
            
for i = 1:(n+1)
    q(1) = limits(1,1) + (i-1)*(limits(1,2)-limits(1,1))/n;
    
    for j = 1:(n+1)
        q(2) = limits(2,1) + (j-1)*(limits(2,2)-limits(2,1))/n;
        
        for k = 1:(n+1)
            q(3) = limits(3,1) + (k-1)*(limits(3,2)-limits(3,1))/n;
            
            for l = 1:(n+1)
                q(4) = limits(4,1) + (l-1)*(limits(4,2)-limits(4,1))/n;

                coords = Forward_Kinematics(q);

                x = coords(1,6);
                y = coords(2,6);
                z = coords(3,6);

                plot3(x,y,z,'bx');
                
            end
        end
    end
end

%% Plot

xlabel('x');
ylabel('y');
zlabel('z');

hold off;
set(h,'visible','on');