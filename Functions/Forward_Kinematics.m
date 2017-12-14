% Forward Kinematics function
%----------------------------------------------------------------------
% Inputs
%   q = [5x1 array] 
%     = all joint angles(deg)
%
% Outputs
%   coords = [3x6 array]
%          = [x;y;z] coordinates of all joints
%----------------------------------------------------------------------
function coords = Forward_Kinematics(q)

% Convert q from deg to rad
    q = q*pi/180;
    
% Change zero position of q4 to horizontal
    q(4) = -pi/2 + q(4);
    
% Load link lengths
    load('stored_variables/link_lengths.mat');
    l = link_lengths;
    

%% DH Matrix

% Joint angles inputted into DH table
    DH = [    0,  pi/2, l(1), q(1);
           l(2),     0,    0, q(2);
           l(3),     0,    0, q(3);
              0, -pi/2,    0, q(4);
              0,     0, l(4), q(5)];
          

%% Transformation matrices

% Calculate transformation matrices between each joint
    for i = 1:5
        T{i} = DHTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        
        if i == 1
            R{i} = T{i};
        else
            R{i} = R{i-1}*T{i};
        end
    end

% Gives coordinates to the middle of the end of each link
    
    coords = [0; 0; 0];
    
    for j = 1:5
        for k = 1:3
            coords(k, j+1) = round(R{j}(k,4), 2);
        end
    end
end