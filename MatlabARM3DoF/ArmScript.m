%Script to run the arm and plot stuff

%Adam Barber
%clear
%Set up parameters, may be necessary
Parameters;

%Set up max time
T = 3;
% %dt is 1ms
dt = 0.001;
% %Set up time vector
t = 0:dt:T;
num_pts = length(t);

% % Right now, want no current for motor 1, and want motor 2 to go from 0 to
% % 0.5 amps. No current for motor 3, but its also not hooked up right now
xDes = zeros(1, num_pts); %motor 1
% xDes = linspace(0,-1.0,num_pts); 
% yDes = linspace(0,-0.6,num_pts); %motor 2
yDes = zeros(1,num_pts);
% thDes = zeros(1, num_pts); %Motor 3
thDes = linspace(0,0.25,num_pts);

% For now, want to go to a home position, and then do a few circles

% xCenter = -0.14;%home position
% radius = 0.05;
% yCenter = -0.28;
% nCircles = 3;
% xDes = xCenter + radius*sin(t*nCircles/T*2*pi);
% 
% yDes = yCenter + radius*cos(t*nCircles/T*2*pi);
% 
% thDes = zeros(size(xDes));
armConfig = 'ELBOW_DOWN';

% T = 0.1;
% dt = 0.001;
% t = 0:dt:T;
% armConfig = 'ELBOW_UP';
% xDes = -0.3*ones(size(t));
% yDes = -10*t.^2;
% thDes = ones(size(xDes))*pi/2;
%ELBOW UP means UP for positive theta1, negative theta2?
%Not quite sure what it means, but with the way i've set the home position
%its in 'elbow down' configuration.

% T = 0.1;
% dt = 0.001;
% t = 0:dt:T;
% armConfig = 'ELBOW_UP';
% xDes = -0.3*ones(size(t));
% yDes = 10*t.^2 - 0.1;
% thDes = ones(size(xDes))*0.0;

% T = 0.12;
% dt = 0.001;
% t = 0:dt:T;
% armConfig = 'ELBOW_UP';
% xDes = -0.35*ones(size(t));
% yDes = (1/.12)*t.^2 - 0.12;
% thDes = ones(size(xDes))*0.0;


%NOW THEY MATTER!
% kp = [250 200 0];
% kd = [4 2 0];
% ki = [1 1 0];
% kp = [252.8989 555.5506 0];
% kd = [1.9375 0.9688 0];
% ki = [0.4844 0.4844 0];
% Previous best!
% kp = [300 450 20];
% kd = [10 15 3];
% ki = [0 0 0];
%Screwin around
kp = [300 450 30];
kd = [10 15 3];
ki = [0 0 0];

%Send it, do it, get it back
[pos1 pos2 pos3 th1 th2 th3 con1 con2 con3] = ...
    sendTrajectory(xDes, yDes, thDes, t, armConfig, kp, kd, ki, params);

%Plot it
plotTrajectory(t, th1, th2, th3, pos1, pos2, pos3, con1, con2, con3, params);
