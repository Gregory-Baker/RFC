% Parallel Robot IK function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   x - x pos of centre of platform (mm)
%   y - y pos of centre of platform (mm)
%   alpha - angle of end effector (deg)
%
% Output
%   coords [x, y] - same as input coords if reachable
%          [0, 0] - if unreachable
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function in_workspace = parallel_IK_func(x_c, y_c, alpha)
    
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

        c_phi_2 = (x_pp(i) - S_A*cos(theta_2))/L;
        s_phi_2 = (y_pp(i) - S_A*sin(theta_2))/L;

        if imag(c_phi_1) ~= 0 && imag(c_phi_2) ~= 0
            in_workspace = false;
            return
        end
    end
    in_workspace = true;
end

