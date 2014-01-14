%This function will take in a desired object trajectory in the one-point
%roll and will return the desired trajectory along with the manipulator
%angle that will minimize the required friction coefficient (align the
%normal vector with the contact force vector)

function xd = one_point_roll_min_mu(xdObject, tdObject, params)
%Pull out useful params
g = params.g; thc = params.objAngle; mo = params.mo;
Jo = params.Jo;

xVel = derivative(xdObject(1,:))./derivative(tdObject);
yVel = derivative(xdObject(3,:))./derivative(tdObject);
thVel = derivative(xdObject(5,:))./derivative(tdObject);
xAcc = derivative(xVel)./derivative(tdObject);
yAcc = derivative(yVel)./derivative(tdObject);
thAcc = derivative(thVel)./derivative(tdObject);

Fx = mo*xAcc;
Fy = mo*(yAcc + g);
F_angle = atan2(Fy,Fx);
thmDes = F_angle - pi/2;
dt = tdObject(2) - tdObject(1);
RC = 1/(1*2*pi);
alpha = dt/(RC+dt);
for i = 2:length(thmDes)
    thmDes(i) = alpha*thmDes(i) + (1-alpha)*thmDes(i-1);
end
thmDesDot = derivative(thmDes)./derivative(tdObject);
xd = [thmDes;thmDesDot;xdObject];

%Just check and make sure that tho works...
thoddCheck = (Fx.*sin(xdObject(5,:) + thc) - ...
    Fy.*cos(xdObject(5,:) + thc))/Jo;
% plot(tdObject,thAcc,'-r',tdObject,thoddCheck,'-b')
end