clear all
close all
clc

p(1,:) = [150, -100, 200];
p(2,:) = [150, 100, 200];

% Location and radius of spherical object
 h = 150;
 j = -50;
 k = 200
 r = 20;

%% Straight line to target

n = 20;
pitch = 0;

x = zeros(1,20);
y = zeros(1,20);
z = zeros(1,20);

i = 1;

x(i) = p(1,1);
y(i) = p(1,2);
z(i) = p(1,3);

[xs,ys,zs] = sphere;

figure(1)
grid on
hold on
surf(xs*r+h, ys*r+j, zs*r+k)
surf(xs*r+h, ys*r-j, zs*r+k)
view([80 30])
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-100 200])
ylim([-150 150])
zlim([0  250])
plot3(p(2,1), p(2,2), p(2,3), 'k*', 'MarkerSize', 12)
text(p(2,1), p(2,2)+10, p(2,3), 'Target', 'FontSize', 12)

plot3(p(1,1), p(1,2), p(1,3), 'k*', 'MarkerSize', 12)
text(p(1,1), p(1,2)-10, p(1,3), 'Start', 'FontSize', 12, 'HorizontalALignment', 'right')

while round(x(i)) ~= p(2,1) || round(y(i)) ~= p(2,2) || round(z(i) ~= p(2,3))
    % Matrix of sampling points
        x(:) = linspace(x(i), p(2,1), n);
        y(:) = linspace(y(i), p(2,2), n);
        z(:) = linspace(z(i), p(2,3), n);

    for i = 1:n

        if (x(i)-h)^2 + (y(i)-j)^2 + (z(i)-k)^2 < (r+5)^2 || (x(i)-h)^2 + (y(i)+j)^2 + (z(i)-k)^2 < (r+5)^2
            random = rand(1);
            if random <= (1/2)
                z(i) = z(i) + 5;
            else
                x(i) = x(i) - 5;
            end
            break
        end

        joint_angles = inverse_kinematics(x(i),y(i),z(i),pitch);

        coords = Forward_Kinematics(joint_angles);

        h2 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
        pause(0.2)

    end
    
end

disp('done')
    