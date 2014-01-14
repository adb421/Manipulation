%
clear classes
dt = 0.001;
params = ParametersFunction();

%Create a pc104 object & connect
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();
% For trajectory
% % %Need to create a trajectory first
% T = 10;
% tPoly = 0:0.001:T;
% xo = -0.3; xf = -0.1;
% yo = 0.1; yf = 0.0;
% %3rd order polynomials that have zero start and end velocity
% ax = xo; bx = 0; cx = 3*(xf - xo)/T^2; dx = 2*(xo - xf)/T^3;
% ay = yo; by = 0; cy = 3*(yf - yo)/T^2; dy = 2*(yo - yf)/T^3;
% xPoly = [dx cx bx ax];
% yPoly = [dy cy by ay];
% t1 = 0:0.001:2.499;
% t2 = 2.5:0.001:T+2.5;
% t3 = T+2.501:0.001:T+5;
% t = [t1 t2 t3];
% % t = tPoly;
% num_pts = length(t);
% xd = [zeros(1,length(t));
%       ones(1,length(t1))*xo polyval(xPoly,tPoly) ones(1,length(t3))*xf;
%       ones(1,length(t1))*yo polyval(yPoly,tPoly) ones(1,length(t3))*yf;
%       ones(1,length(t))*(pi/2-params.objAngle)];
% % xd = [zeros(1,num_pts); polyval(xPoly,t); ...
% %     polyval(yPoly,t); ones(1,num_pts)*(pi/2-params.objAngle)];
% 
% %Try ud = 0? if not, whats happening in the simulator
% ud = zeros(3,length(t));
% % ud = [polyval(polyder(polyder(xPoly)),tPoly); ...
% %     polyval(polyder(polyder(yPoly)),tPoly); ...
% %     zeros(1,length(tPoly))];
% 
% home = [xo; yo - params.wm - params.lc; 0];

% % For equilibrium
% T = 10;
% t = 0:0.001:T;
% num_pts = length(t);
% xd = zeros(4,num_pts);
% xd(2,:) = (-0.2)*ones(1,num_pts);
% xd(3,:) = (-0.1 + params.wm + params.lc)*ones(1,num_pts);
% % xd(2,:) = -0.3*ones(1,num_pts);
% % xd(3,:) = 0.1*ones(1,num_pts);
% xd(4,:) = (pi/2 - params.objAngle)*ones(1,num_pts);
% % home = [-0.2, -0.1, 0];
% home = [-0.2; -0.1; 0];
% ud = zeros(3,num_pts);
% pc104.sendHomePos(home);

% Balance then a "throwing" trajectory
T = 0.75;
tBal = -5:0.001:0;
tPoly = 0.001:0.001:T;
t = [tBal, tPoly];
num_pts = length(t);
xo = -0.275; yo = -0.1 + params.lc + params.wm; yf = yo + 0.2;
yThrowVel = 0.8;
% d = (T*yThrowVel - 2*yf + 2*yo)/T^3;
% c = -(T*yThrowVel - 3*yf + 3*yo)/T^2;
c = yThrowVel/(2*T);
b = 0; a = yo;
yPoly = [c b a];
xd = [zeros(1,num_pts); ...
      ones(1,num_pts)*xo; ...
      ones(1,length(tBal))*yo polyval(yPoly,tPoly); ...
      ones(1,num_pts)*(pi/2 - params.objAngle)];
ud = [derivative(xd(2,:))./derivative(t);...
      derivative(xd(3,:))./derivative(t);...
      derivative(xd(1,:))./derivative(t)];
home = [xd(2,1); xd(3,1) - params.wm - params.lc; 0];
t = t+5;
pc104.sendHomePos(home);

disp('unpause to go home')
pause
pc104.goHome();

pc104.allocateTraj(num_pts);
pc104.LQRTrajSend(xd(1,:),xd(2,:),xd(3,:),xd(4,:),ud(1,:),ud(2,:),ud(3,:),t);
pause(10);
disp('unpause for traj go')
pause
pc104.goTraj();
disp('Going')
pause(t(end)+1)
pc104.getTrajData();
pc104.plotTrajData();
pc104.killProgram();