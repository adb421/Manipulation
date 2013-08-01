%%
clear classes
dt = 0.001;
params = ParametersFunction();

%Create pc104 object & connect
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();
%Send gains, LQR gains are calculated by the class
pc104.LQRGainSend();
% pc104.newLQRGainSend();

%Record data for 30 seconds
T = 30;
t = 0:dt:T;
num_pts = length(t);

%Send manipulator home position
home = [-0.2 0.0 0.0];
pc104.sendHomePos(home);

disp('unpause to go home')
pause
pc104.goHome();

%Now send traj...
pc104.allocateTraj(num_pts);
trajs = zeros(size(t));
pc104.sendTraj(trajs,trajs,trajs,t);

disp('unpause for traj go')
pause
% pause(8)
%Execute!
pc104.goTraj();
pause(t(end));
pc104.getTrajData();
pc104.plotTrajData();
pc104.killProgram();
