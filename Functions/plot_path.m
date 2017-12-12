% Plot path of moving arm
%--------------------------------
function plot_path(coords)

% Plot arm
    clf
    grid on
    hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-200 200])
    zlim([-50  250])
    view([50 30])
    plot3(coords(1,:), coords(2,:), coords(3,:));
    pause(0.1)
    hold off
    pause(0.1)
    
end