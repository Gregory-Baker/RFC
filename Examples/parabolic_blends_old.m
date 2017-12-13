%% Old version of parabolic blends

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