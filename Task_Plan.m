% Plan A Task

clear all
close all

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
    
% Time interval between waypoints
    time_int = T/(n_points-1);
    
% Number of samples between each point
    n = 10;
    

%% Plot 5 positions

% Initiate Figure
    figure(1)
    hold on    
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view([80 30])
    grid on
    run('plot_positions.m')
    plot3(poe(1,:), poe(2,:), poe(3,:), 'k--')
    hold off
    
    
%% Free motion trajectory with parabolic blends


% Define global acceleration (deg/sec^2)
    accn_mag = 30
        
for n = 1:4
    
    t = [0; 0; 0; 0];

    for j = 1:(n_points-1)

        theta_1 = joint_angles_all(j,n);
        theta_2 = joint_angles_all(j+1,n);
        theta_diff = theta_2-theta_1;

        accn_sign = sign(theta_diff);
        accn = accn_sign*accn_mag;

        vel1 = (time_int*accn - sqrt((time_int*accn)^2-4*accn*theta_diff))/2;
        vel2 = (time_int*accn + sqrt((time_int*accn)^2-4*accn*theta_diff))/2;

        if accn > 0
            vel = min(vel1, vel2);
        else
            vel = max(vel1, vel2);
        end

        tb(j) = vel/accn;

        theta{n}(j,1) = theta_1;
        velocity{n}(j,1) = 0;
        acceleration{n}(j,1) = accn;

        n_samples = 101;

        for i = 2:n_samples

            t(i) = time_int*(i-1)/(n_samples-1);

            if t(i) < tb(j)     
                acceleration{n}(j,i) = accn;
                velocity{n}(j,i) = accn*t(i);
            elseif t(i) > (time_int - tb(j))   
                acceleration{n}(j,i) = -accn;
                velocity{n}(j,i) = vel - (t(i)- (time_int-tb(j)))*accn;
            else
                acceleration{n}(j,i) = 0;
                velocity{n}(j,i) = vel;
            end

            theta{n}(j,i) = theta{n}(j,i-1) + velocity{n}(j,i)*(t(i)-t(i-1)) + (acceleration{n}(j,i)*(t(i)-t(i-1))^2)/2;
            
        end
    end
    
    
    % Reshape matrices into arrays containing all points
    m = 1;
    time_vec(1) = 0;
    time_step = time_int/n_samples;
    
    for j = 1:(n_points-1)
        for i = 1:n_samples
            theta_vec(n,m)        = theta{n}(j,i);
            velocity_vec(n,m)     = velocity{n}(j,i);
            acceleration_vec(n,m) = acceleration{n}(j,i);
            if m ~= 1
                time_vec(m)     = time_vec(m-1)+time_step;
            end
            m = m + 1;
        end
    end
    theta_vec(n,m)        = theta{n}(j,i);
    velocity_vec(n,m)     = velocity{n}(j,i);
    acceleration_vec(n,m) = acceleration{n}(j,i);
    time_vec(m)     = time_vec(m-1)+time_step;
end


for f = 1:m
   hold on
   q_free = [theta_vec(1,f), theta_vec(2,f), theta_vec(3,f), theta_vec(4,f), 0];
   coords = Forward_Kinematics(q_free);
   poe_free(:,f) = [coords(1,6); coords(2,6); coords(3,6)];
end

figure(8)
grid on
hold on
run('plot_positions.m')
plot3(poe_free(1,:), poe_free(2,:), poe_free(3,:), 'k--');
view([80 30])
xlabel('x')
ylabel('y')
zlabel('z')
hold off

figure(5)
plot(time_vec, theta_vec);
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');

figure(6)
plot(time_vec, velocity_vec);
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');

figure(7)
plot(time_vec, acceleration_vec);
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4');
    


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
    
% Plot end effector points
    figure(3)
    hold on
    plot3(x2, y2, z2, 'bx')
    plot3(x2, y2, z2, 'k')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view(3)
    grid on
    hold off
    
% Plot arm position throughout task


% Calculate joint angles for all sampled points using IK 
for i = 1:(n_points-1)*(n-1)+1

    joint_angles = inverse_kinematics(x2(i), y2(i), z2(i), pitch2(i));
    
% Coordinates of links from FK
    coords = Forward_Kinematics(joint_angles);

% Plot arm trajectory
    % figure(4)
    % plot_path(coords);
        
end







