%--------------------------------------------------------------
% Inputs
%   joint_angles = array of joint angles to go via (deg)
%   accn_mag = max acceleration of arm (deg/sec)
%   total_time = total time for manouevre (sec)
% 
% Outputs
%   joint_values = [4x1001 array], rows:
%        time               (sec)
%        joint_angle        (deg)
%        joint_velocity     (deg/sec)
%        joint_acceleration (deg/sec^2)
%%--------------------------------------------------------------
function joint_values = parabolic_blend(joint_angles, accn_mag, time_total)

% The number of points in the task
    n_points = size(joint_angles, 2);

% The number of transitions between points
    n_transitions = n_points - 1;

% Duration of each transition
    time_inc = time_total/(n_transitions); 

% Calculate angle differences
    for i = 1:n_transitions
        theta_diff(i) = joint_angles(i+1) - joint_angles(i);
    end

% Acceleration at start and end
    accn(1) = sign(theta_diff(1))*accn_mag;
    accn(n_points) = sign(-theta_diff(n_transitions))*accn_mag;

% Duration of acceleration at start and end
    t(1) = time_inc - sqrt(time_inc^2 - 2*theta_diff(1)/accn(1));
    t(n_points) = time_inc - sqrt(time_inc^2 + 2*theta_diff(n_transitions)/accn(n_points));

% Linear section velocities of first and last transition
    vel(1) = theta_diff(1)/(time_inc-0.5*t(1));
    vel(n_transitions) = theta_diff(n_transitions)/(time_inc - 0.5*t(n_points));

% Linear section velocities of middle transitions
    for i = 2:(n_transitions-1)
        vel(i) = theta_diff(i)/time_inc;
    end
    
% Accelerations at each joint
    for i = 2:n_transitions
        accn(i) = sign(vel(i)- vel(i-1))*accn_mag;
    end
    
% Duration of acceleration at each point
    for i = 2:n_transitions
        t(i) = (vel(i)-vel(i-1))/accn(i);
    end

% Duration of linear velocity sections
    t_lin(1) = time_inc - t(1) - 0.5*t(2);
    t_lin(n_transitions) = time_inc - t(n_points) - 0.5*t(n_transitions);
    
    for i = 2:(n_transitions-1)
        t_lin(i)= time_inc - 0.5*t(i)-0.5*t(i+1);
    end

% Counter variable for time mode
    j = 2;
    
% Time of switches between acceleration modes and linear velocity modes
    time_mode(1) = 0;

% Add mode switch time for each point to time_mode
    for i = 1:n_points

        time_mode(j) = time_mode(j-1) + t(i);
        
    % Variable defines when acceleration is occuring
        accelerating(j) = 1;

        j = j+1;

        if i ~= n_points
            accelerating(j) = 0;
            time_mode(j) = time_mode(j-1)+ t_lin(i);
        end
        j = j+1;
    end

    time(1) = 0;
    
% Number of samples
    n_inc = 1001;

% Increment of time between each sample
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

    joint_values(1,:) = time(:);
    joint_values(2,:) = theta(:);
    joint_values(3,:) = velocity(:);
    joint_values(4,:) = acceleration(:);
    
end

