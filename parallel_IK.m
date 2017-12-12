% Parallel Robot

%clear all
%close all

%% Input Parameters

x_c = 0;     % x pos of centre of platform (mm)
y_c = 0;    % y pos of centre of platform (mm)
alpha = 0;     % Angle of platform (deg)

alpha = alpha*pi/180;
c_a = cos(alpha);
s_a = sin(alpha);


%% Design Parameters

S_A = 170;     % Length Base to elbow link (mm)
L   = 130;     % Length elbow to platform link (mm)
r_p = 130;     % Joint circle radius, platform (mm)
r_b = 290;     % Joint circle radius, base (mm)

%% Coordinates of joints

PB = [pi/2, pi+pi/6, 2*pi-pi/6];

PP = [pi/2, pi+pi/6, 2*pi-pi/6];

%% Rotation Matrices

for i = 1:3

    R_BC = [c_a, -s_a, 0;
            s_a,  c_a, 0;
              0,    0, 1];

    CPP = [cos(PP(i))*r_p;
           sin(PP(i))*r_p;
                 0       ];

    BC = [x_c;
          y_c;
            0];

    BPP = R_BC*CPP+BC;

    BPB = [cos(PB(i))*r_b;
              sin(PB(i))*r_b;
                    0       ];

    PBPP = BPP - BPB;

    x_pp(i) = PBPP(1,1);
    y_pp(i) = PBPP(2,1);
    
% Plotting points
    base_plot(i,1) = BPB(1,1);
    base_plot(i,2) = BPB(2,1);
    
    platform_plot(i,1) = BPP(1,1);
    platform_plot(i,2) = BPP(2,1);

% Relationship between base and platform
    e1 = -2 * y_pp(i) * S_A;
    e2 = -2 * x_pp(i) * S_A;
    e3 = x_pp(i)^2 + y_pp(i)^2 + S_A^2 - L^2;

    t_1 = (-e1 + sqrt(e1^2 + e2^2 - e3^2)) / (e3 - e2);
    t_2 = (-e1 - sqrt(e1^2 + e2^2 - e3^2)) / (e3 - e2);

    theta_1 = 2*atan(t_1);
    theta_2 = 2*atan(t_2);
    
% Position of Midpoint
    M_1(i,1) = BPB(1,1) + S_A*cos(theta_1);
    M_1(i,2) = BPB(2,1) + S_A*sin(theta_1);
    
    M_2(i,1) = BPB(1,1) + S_A*cos(theta_2);
    M_2(i,2) = BPB(2,1) + S_A*sin(theta_2);
    
% Following Arm Angle
    c_phi_1 = (x_pp(i) - S_A*cos(theta_1))/L;
    s_phi_1 = (y_pp(i)- S_A*sin(theta_1))/L;

    phi_1 = atan2( s_phi_1, c_phi_1 );

    c_phi_2 = (x_pp(i) - S_A*cos(theta_2))/L;
    s_phi_2 = (y_pp(i) - S_A*sin(theta_2))/L;
    phi_2 = atan2( s_phi_2, c_phi_2 );
   
    sol{i} = [theta_1*180/pi, phi_1*180/pi;
              theta_2*180/pi, phi_2*180/pi];

    Leg_no = ['Leg ', num2str(i), ':'];
    disp(Leg_no)
    disp('Solution 1 =')
    disp(sol{i}(1,:))

    disp('Solution 2 =')
    disp(sol{i}(2,:))
end

%% Plotting

% Complete the plot

base_plot(4,1) = base_plot(1,1);
base_plot(4,2) = base_plot(1,2);

platform_plot(4,1) = platform_plot(1,1);
platform_plot(4,2) = platform_plot(1,2);

for i = 1:3
    arm{i}(1,:) = base_plot(i,:);
    arm{i}(2,:) = M_1(i,:);
    arm{i}(3,:) = platform_plot(i,:);
    
    hold on
    plot(arm{i}(:,1),arm{i}(:,2),'b')
end

plot(base_plot(:,1),base_plot(:,2),'k')
plot(platform_plot(:,1),platform_plot(:,2),'r')
axis equal
hold off


