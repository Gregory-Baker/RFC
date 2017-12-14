clear all
close all

% Obstacle avoidance in task

p(1,:) = [6, 46, 70, -136];
p(2,:) = [129, 150, 150, 30];
p(3,:) = [150, 80, 70, -20];
p(4,:) = [136, -69, -67, -78];
p(5,:) = [60, -146, 170, 36];


% Location and radius of spherical objects (mm)
    h = (p(3,1)+p(4,1))/2;
    j = (p(3,2)+p(4,2))/2;  
    k = (p(3,3)+p(4,3))/2;
    r = 30;
    
% Number of points
    n_points = size(p,1);
    
% Number of points between start point and target
    n = 100;
    
% Index variables
    i = 1;
    m = 1;

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
    ylim([-200 200])
    zlim([-100  250])
    
% Plot spherical obstacles
    surf(xs*r+h, ys*r+j, zs*r+k)

for i = 1:(n_points-1)
    
        a = 1;
    
        x_int = zeros(1,n);
        y_int = zeros(1,n);
        z_int = zeros(1,n);
        pitch_int = zeros(1,n);
    
        x_int(a) = p(i,1);
        y_int(a) = p(i,2);
        z_int(a) = p(i,3);
        pitch_int(a) = p(i,4);
        
   while round(x_int(a)) ~= p(i+1,1) || round(y_int(a)) ~= p(i+1,2) || round(z_int(a) ~= p(i+1,3)) || round(pitch_int(a) ~= p(i+1,4))         
        
        x_int(:) = linspace(x_int(a), p(i+1,1), n);
        y_int(:) = linspace(y_int(a), p(i+1,2), n);
        z_int(:) = linspace(z_int(a), p(i+1,3), n);
        pitch_int(:) = linspace(pitch_int(a), p(i+1, 4), n);
        
        for a = 1:n
            if (x_int(a)-h)^2 + (y_int(a)-j)^2 + (z_int(a)-k)^2 < (r+6)^2
                random = rand(1);
                if random <= (1/2)
                    z_int(a) = z_int(a) - 5;
                else
                    x_int(a) = x_int(a) - 5;
                end
                break
            end
            joint_angles = inverse_kinematics(x_int(a),y_int(a),z_int(a),pitch_int(a));

            coords = Forward_Kinematics(joint_angles);

            for l = 1:3
                poe(l,m) = coords(l,6);
            end

            if exist('h2')
                set(h2,'Visible','off')
            end
            h2 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
            pause(0.02)
            plot3(poe(1,:), poe(2,:), poe(3,:), 'r--', 'LineWidth', 2)
            m = m+1;
           
        end
        
    end 
end