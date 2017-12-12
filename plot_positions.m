% Plot 5 positions
for i = 1:size(p,1)
    
% Joint angles from IK
    joint_angles = inverse_kinematics (p(i,1),p(i,2),p(i,3),p(i,4));
    joint_angles_all(i,:) = joint_angles;

% Coordinates of links from FK
    coords = Forward_Kinematics(joint_angles);

% Point of end effector
    poe(:,i) = coords(:,6);

% Plot arm position
    plot3(coords(1,:), coords(2,:), coords(3,:))

end