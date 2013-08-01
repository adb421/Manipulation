%Generate a simple trajectory for the arm, and we'll get camera data for it
dt = 0.001;
T = 2;
t = 0:dt:T;

%X = constant
xDes = -0.15*ones(size(t));
%Theta = constant
thDes = zeros(size(t));
%Y go from -0.10 to 0.10 linearly
yPoly = [0.1 -0.1];
yDes = polyval(yPoly,t);