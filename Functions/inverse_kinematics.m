function joint_angles = inverse_kinematics(x, y, z, pitch_deg)

% Position of end effector
    poe = [x y z];

%% Initialise values

% Load link lengths
    load('stored_variables/link_lengths.mat');
    l = link_lengths;

% z value offset by length of first link for future calculations
    z = z - l(1);

% Initialise joint angles 
    q = [0 0; 0 0; 0 0; 0 0; 0 0];

% Convert pitch to radians
    pitch = pitch_deg*pi/180;

%% Calculate Joint Angles

% Calculate q1
    q(1,1) = atan(y/x);
    q(1,2) = q(1,1);

    
%% Find q3

% Sign of y
    if sign(x) == -1 %&& sign(y) == -1
        sign_f = -1;
    else
        sign_f = 1;
    end

% In plane lateral distance of joint 5
    p5 = sign_f*sqrt(x^2 + y^2);

% In plane lateral and vertical distances of joint 4 
    p4 = p5 - l(4)*cos(pitch);
    z4 = z - l(4)*sin(pitch);

% Cosine of joint 3 angle, from formula
    c3 = (z4^2 + p4^2 - l(2)^2 - l(3)^2)/(2*l(2)*l(3));

% Sine values  of joint 3 angle
    s3_1 = sqrt(1 - c3^2);
    s3_2 = -sqrt(1 - c3^2);
    
% Joint 3 angles
    q(3,1) = atan2(s3_1, c3);
    q(3,2) = atan2(s3_2, c3);

%% Find q2

% Define k parameters (as in FK example)
    k1 = l(2)+l(3)*c3;
    k2_1 = l(3)*s3_1;
    k2_2 = l(3)*s3_2;

% Calculate gamas
    gama1 = atan2(k2_1,k1);
    gama2 = atan2(k2_2,k1);

% Calculate r
    r = sqrt(k1^2+k2_1^2);

% Joint 3 angles
    q(2,1) = atan2(z4/r, p4/r) - gama1;
    q(2,2) = atan2(z4/r, p4/r) - gama2;


%% Find q4

% Joint 4 angles, pi/2 term included to change zero pos to horizontal
    q(4,1) = pitch - q(2,1) - q(3,1); 
    q(4,2) = pitch - q(2,2) - q(3,2); 


%% Return Solutions

% Load joint limits
    load('stored_variables/limits.mat');

% Express joint angles in degrees
    q = q*180/pi;

% Loop compares both possible solution to robot angle limits
    for j = 1:2

        % Counter for number of angles outside limits
        out_of_limits = 0;

        for i = 1:4

            if round(q(i,j)) <= limits(i,1) || round(q(i,j)) >= limits(i,2)
                out_of_limits = out_of_limits + 1;
            end
        end

        if out_of_limits == 0
            joint_angles = q(:,j);           
            return
        end
    end
    
    p0e = [x, y, z+l(1)];
    disp(p0e)
    disp(q)
    disp(limits)
    disp('This position is unreachable')

end