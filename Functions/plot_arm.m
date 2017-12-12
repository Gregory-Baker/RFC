function plot_arm(coords)

figure
hold on
plot3(coords(1,:), coords(2,:), coords(3,:))
xlabel('x')
ylabel('y')
zlabel('z')
view(3)
grid on
hold off
    
end


    