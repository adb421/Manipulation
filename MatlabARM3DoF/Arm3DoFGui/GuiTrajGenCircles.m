% Create trajectory variables for the gui.
% Variables need to have the names "xDes", "yDes", "thDes", and "t"

% Let's do circles! I kinda like circles.
% Do 9 circles in 3 seconds. 1ms timesteps
T = 3;
nCircles = 3;
dt = 0.001;
t = 0:dt:T;

%Center the circle at (-0.14,-0.28) with a radius of 5cm
radius = 0.05; xCenter = -0.14; yCenter = -0.28;
%Calculate xDes and yDes, points along the trajectory
xDes = xCenter + radius*sin(t*nCircles/T*2*pi);
yDes = yCenter + radius*cos(t*nCircles/T*2*pi);
%Make this zero, doesn't matter since right now we don't have encoders
thDes = zeros(size(xDes));
%Set arm configuration to elbow down