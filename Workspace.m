% Workspace

clear all
close all

q_1 = 0;
q_2 = 0;
q_3 = 0;
q_4 = 0;
q_5 = 0;

counter = 0;

limits = [-90,  90;
            0, 180;
         -160,   0;
         -130,  90;
            0, 360];
       
n = 8; % Number of steps between limits

h = figure('visible','off');
hold on;
            
for i = 1:(n+1)
    
    q_1 = limits(1,1) + (i-1)*(limits(1,2)-limits(1,1))/n;
    
    for j = 1:(n+1)
        
        q_2 = limits(2,1) + (j-1)*(limits(2,2)-limits(2,1))/n;
        
        for k = 1:(n+1)
            
            q_3 = limits(3,1) + (k-1)*(limits(3,2)-limits(3,1))/n;
            
            for l = 1:(n+1)
                
                q_4 = limits(4,1) + (l-1)*(limits(4,2)-limits(4,1))/n;

                End_Eff = Forward_Kinematics(q_1, q_2, q_3, q_4, q_5);

                x = End_Eff(1,4);
                y = End_Eff(2,4);
                z = End_Eff(3,4);

                plot3(x,y,z,'bx');
                
                counter = counter + 1;
            end
        end
    end
end

xlabel('x');
ylabel('y');
zlabel('z');

hold off;
set(h,'visible','on');