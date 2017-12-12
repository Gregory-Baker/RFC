clear all
close all

time_total = 20;
n_points = 5;
time_inc = time_total/(n_points-1); % [5, 6, 3, 4] % secs

accn_mag = 30; % deg/sec^2

joint_angles = [40, 80, -20, 10, 65];


% Calculate accelerations
for i = 1:4
    
    theta_diff(i) = joint_angles(i+1) - joint_angles(i);
    accn(i) = sign(theta_diff(i))*accn_mag;

end

accn(5) = sign(-theta_diff(4))*accn_mag;

t(1) = time_inc - sqrt(time_inc^2 - 2*theta_diff(1)/accn(1));
t(5) = time_inc - sqrt(time_inc^2 + 2*theta_diff(4)/accn(5));

vel(1) = theta_diff(1)/(time_inc-0.5*t(1));
vel(4) = theta_diff(4)/(time_inc - 0.5*t(5));

% Calculate velocities
for i = 2:4
    
    if i ~= 4
        vel(i) = theta_diff(i)/time_inc;
    end
    
    t(i) = (vel(i)-vel(i-1))/accn(i);
end


% Calculate linear velocity times
t_lin(1) = time_inc - t(1) - 0.5*t(2);
t_lin(4) = time_inc - t(5) - 0.5*t(4);
for i = 2:3
    t_lin(i)= time_inc - 0.5*t(i)-0.5*t(i+1);
end

j = 2;
time_mode(1) = 0;

for i = 1:5
    
    time_mode(j) = time_mode(j-1) + t(i);
    accelerating(j) = 1;
    
    j = j+1;
    
    if i ~= 5
        accelerating(j) = 0;
        time_mode(j) = time_mode(j-1)+ t_lin(i);
    end
    j = j+1;
end

time(1) = 0;
n_inc = 1001;
time_step = time_total/(n_inc-1);

j = 2;
k = 1;
velocity(1) = 0;
acceleration(1) = accn(1);
theta(1) = joint_angles(1);

for i = 2:n_inc
    time(i) = time(i-1) + time_step;
  
    if time(i) > time_mode(j)
        j = j + 1;
        if mod(j,2) == 0
            k = k+1;
        end
    end
    
    acceleration(i) = accelerating(j)*accn(k);
    velocity(i) = velocity(i-1) + acceleration(i-1)*time_step;
    theta(i) = theta(i-1) + (velocity(i-1)*time_step) + (acceleration(i-1)*time_step^2)/2;
    
end

figure()
plot(time, theta);

figure()
plot(time, velocity);

figure()
plot(time, acceleration);
