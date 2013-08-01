% Create trajectory variables for the gui.
% Variables need to have the names "xDes", "yDes", "thDes", and "t"

%Let's do a throwing motion
%Goal velocity at the end:
% yVelEnd = 3; %m/s
% yStart = -0.1; %m
% yThrow = 0.1;
% T = 0.5;
% dt = 0.001;
% t = 0:dt:T;
% % %Define coefficients for polynomials, at least to start.
% a = 16*yVelEnd/T^4;
% b = -8*(2*yStart - 2*yThrow + 5*T*yVelEnd)/T^4;
% c = 32*(yStart - yThrow + T*yVelEnd)/T^3;
% d = -8*(2*yStart - 2*yThrow + T*yVelEnd)/T^2;
% e = 0;
% f = yStart;
% yPoly = [a b c d e f];
% yVelPoly = polyder(yPoly);
% yAccPoly = polyder(yVelPoly);
% yDes = polyval(yPoly,t);
% yVelDes = polyval(yVelPoly,t);
% yAccDes = polyval(yAccPoly,t);
% 
% xDes = -0.27*ones(size(yDes));
% % thDes = pi/2*ones(size(yDes));
% thDes = zeros(size(yDes));

%We can try this at first

%Also try just going straight back.

%New polynomial, lets see if its better

% Try the following
dt = 0.001;
T = 0.8;
t = 0:dt:T;
tThrow = 0.3;
y0 = -0.10;
yThrow = 0.10;
yVelThrow = 1.5;

a = (-(2*T-5*tThrow)*(y0-yThrow) + ...
    tThrow*(tThrow-T)*yVelThrow)/ ...
    ((T-tThrow)^4*tThrow^3);
b = (6*(T^2 - 2*T*tThrow - tThrow^2)*(y0-yThrow) +  ...
    (T-tThrow)*tThrow*(3*T + tThrow)*yVelThrow)/...
    ((T-tThrow)^4*tThrow^3);
c = (-6*T*(T^2 - T*tThrow - 3*tThrow^2)*(y0-yThrow) + ...
    3*T*tThrow*(tThrow^2 - T^2)*yVelThrow)/ ...
    ((T-tThrow)^4*tThrow^3);
d = (T^2*(2*(T^2 + 2*T*tThrow - 9*tThrow^2)*(y0-yThrow) + ...
    (T - tThrow)*tThrow*(T + 3*tThrow)*yVelThrow))/...
    ((T-tThrow)^4*tThrow^3);
e = (T^3*(-3*(T-2*tThrow)*(y0-yThrow) + ...
    tThrow*(tThrow-T)*yVelThrow))/ ...
    ((T-tThrow)^4*tThrow^2);
f = 0;
g = y0;
yPoly = [a b c d e f g];
yDes = polyval(yPoly,t);

yVelPoly = polyder(yPoly);
yAccPoly = polyder(yVelPoly);

yVelDes = polyval(yVelPoly,t);
yAccDes = polyval(yAccPoly,t);

xDes = -0.15*ones(size(yDes));
thDes = zeros(size(yDes));