%Test generate trajectory

L1 = 0.193675;
L2 = 0.19685;

off_x = -L1*sin(pi/3);
off_y = -(L1/2)+L2;

T = 3;
dt = 0.001;

t = 0:dt:(T-dt);
array_length = (T/dt);
%fx = 1; fy = 1;
%phi_x = 0; phi_y = -pi/2;
A_x = .03; A_y = .03;

x = zeros(array_length,1);
y = zeros(array_length,1);
theta = zeros(array_length,1);

for i = 1:array_length
    x(i) = -A_x*cos(2*pi*t(i)) + off_x;
    y(i) = -A_y*sin(2*pi*t(i)) + off_y;
end
plot(t,x)
figure
plot(t,y)
figure
plot(x,y)

% plot(x, y);
% axis square
% figure
% plot(-x,-y);
%[theta1 theta2 theta3] = generateTrajectory(x,y,theta,t,dt,'ELBOW_UP');