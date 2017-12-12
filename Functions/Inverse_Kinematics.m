% Inverse Kinematics

clear all
close all

%% Link Lengths

l1 = 63;
l2 = 153;
l3 = 153;
l4 = 68;

%% Desired position of End Effector (mm) and Pitch of End Effector (deg)

x_e = 200; 
y_e = 200;
z_e = 150;
pitch_deg = 60;

p0e = [ x_e y_e z_e ]' ;
disp('Desired position =')
disp(p0e)

z_e = z_e - l1;
q5 = 0;

pitch = pitch_deg*pi/180;


%% Find q1

q1 = atan2(y_e,x_e);

%% Find q3

p5 = sqrt(x_e^2 + y_e^2);
p4 = p5 - l4*cos(pitch);

z4 = z_e - l4*sin(pitch);

c3 = (z4^2 + p4^2 - l2^2 - l3^2)/(2*l2*l3);

s3_1 = sqrt(1 - c3^2);
s3_2 = -sqrt(1 - c3^2);

q3_1 = atan2(s3_1, c3);
q3_2 = atan2(s3_2, c3);

%% Find q2

k1 = l2+l3*c3;
k2_1 = l3*s3_1;
k2_2 = l3*s3_2;

gama1 = atan2(k2_1,k1);
gama2 = atan2(k2_2,k1);
r = sqrt(k1^2+k2_1^2);

q2_1 = atan2(z4/r, p4/r) - gama1;
q2_2 = atan2(z4/r, p4/r) - gama2;


%% Find q4

q4_1 = pitch - q2_1 - q3_1;
q4_2 = pitch - q2_2 - q3_2;
       

%% Show Solution

disp('solution 1')
disp([ q1 q2_1 q3_1 q4_1 ]*180/pi)
plot_arm(q1, q2_1, q3_1, q4_1, q5)

disp('solution 2')
disp([ q1 q2_2 q3_2 q4_2 ]*180/pi)
plot_arm(q1, q2_2, q3_2, q4_2, q5)





