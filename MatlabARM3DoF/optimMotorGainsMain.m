%This script will attempt to find good control gains for the 3DoF arm by
%using fmincon (maybe fminsearch, but need to make sure control gains are
%positive)

%We'll steal some things from "ArmScript.m"

%Max time
T = 3; %s
%dt is 1ms
dt = 0.001; %s
%Set up time vector
t = 0:dt:T;
num_pts = length(t);

% For now, want to go to a home position, and then do a few circles
xStart = -0.1677;%Home position
xCenter = -0.1677;%home position
radius = 0.025;
yStart = -0.0969;
yCenter = -0.3188;
nCircles = 1; %Want this to go slower
xDes = xCenter + radius*sin(t*nCircles/T*2*pi);

yDes = yCenter + radius*cos(t*nCircles/T*2*pi);

thDes = zeros(size(xDes));

% x0 = [10; 5; 3; 1; 1; 1];
%kp1 kp2 kd1 kd2 ki1 ki2
x0 = [200; 150; 4; 2; 1; 1];

minFxn = ...
    @(x) optimMotorGainsCost(x,xDes, yDes, thDes, t, 'ELBOW_DOWN', params);
lb = [0; 0; 0; 0; 0; 0];
ub = [Inf; Inf; 6; 6; 30; 30];

opts = optimset('Algorithm', 'active-set', 'Display', 'iter', ...
    'MaxFunEvals', 800, 'TolFun', 1e-8, 'TolX', 1e-4);

xSol = fmincon(minFxn, x0, [], [], [], [], lb, ub, [], opts);