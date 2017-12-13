% Plot path of moving arm
%--------------------------------
function plot_path(coords)

% Plot arm
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-100 200])
    ylim([-200 200])
    zlim([-100  250])
    view([50 30])
    check_ex = exist('h2')
    if graph == 1
        set(h2,'Visible','off')
    end
    h2 = plot3(coords(1,:), coords(2,:), coords(3,:), 'b', 'LineWidth', 2);
    pause(0.1)
end