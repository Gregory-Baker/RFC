% Parallel Robot Workspace

clear all
close all

alpha = 0;

xmin = -200;
xmax = 200;

ymin = -150;
ymax = 200;

alpha_min = -45;
alpha_max = 45;

n = 100;
n_alpha = 30;

figure(1)
hold on
%Iterate through all potential points and angles
    for i = 1:(n+1)
        x = xmin+(i-1)*(xmax-xmin)/n;
        for j = 1:(n+1)
            y = ymin+(j-1)*(ymax-ymin)/n;
            for k = 1:(n_alpha+1)
                alpha = alpha_min+(k-1)*(alpha_max-alpha_min)/n_alpha;
                
                in_workspace = parallel_IK_func(x,y,alpha);
                if in_workspace
                    plot(x, y, 'c.');
                end
            end
        end
    end
run('parallel_IK.m')
axis equal