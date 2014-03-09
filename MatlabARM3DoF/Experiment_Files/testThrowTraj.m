%A script to do a "throw" motion from awhile ago. This can be used to
%demonstrate the manipulator trajectory following mode, FF_PID_TRAJ_MANIP
%(8). This shows how a trajectory could be made in cartesian space for the
%manipulator and used on the robot.
clear classes
params = ParametersFunction();
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();
pc104.sendControlGains();
T = 0.75;
t = 0:0.001:T;
num_pts = length(t);
xo = -0.275;
yo = -0.1;

yThrowVel = 0.8;
c = yThrowVel/(2*T);
b = 0; a = yo;
yPoly = [c b a];
xd = [ones(1,num_pts)*xo; polyval(yPoly, t); zeros(1,num_pts)];
home = xd(:,1);
pc104.sendHomePos(home);
disp('unpause to go home')
pause
pc104.goHome();

%Send traj
pc104.allocateTraj(num_pts);
pc104.setTrajectoryControlMode(params.FF_PID_TRAJ_MANIP);
pc104.sendTraj(xd(1,:),xd(2,:),xd(3,:),t);

disp('unpause for traj go')
pause
pc104.goTraj();
pause(t(end));
pc104.getTrajData();
pc104.plotTrajData();
pc104.killProgram();