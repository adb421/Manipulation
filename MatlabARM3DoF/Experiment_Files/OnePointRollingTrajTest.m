% Double check
clear classes
dt = 0.001;
params = ParametersFunction();

%Create a pc104 object & connect
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();


% Balance then a "throwing" trajectory
% Done with a polynomial throwing trajectory.
T = 0.75;
tBal = -5:0.001:0;
tPoly = 0.001:0.001:T;
t = [tBal, tPoly];
num_pts = length(t);
xo = -0.275; yo = -0.1 + params.lc + params.wm; yf = yo + 0.2;
yThrowVel = 0.8;
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
pc104.setTrajectoryControlMode(params.ONE_POINT_ROLL_TRAJ);
pause(10);
disp('unpause for traj go')
pause
pc104.goTraj();
disp('Going')
pause(t(end)+1)
pc104.getTrajData();
pc104.plotTrajData();
pc104.killProgram();