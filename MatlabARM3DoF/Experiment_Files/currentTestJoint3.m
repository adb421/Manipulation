
dt = 0.001;
params = ParametersFunction;
T = 30;
t = 0:dt:T;

pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();

%Current trajectory
traj1 = zeros(size(t));
traj2 = zeros(size(t));
traj3 = .8*ones(size(t));
num_pts = length(t);

pc104.allocateTraj(num_pts);
pc104.sendTraj(traj1, traj2, traj3, t);
pause()
pc104.goTraj();
pause(T);
pc104.getTrajData();
pc104.killProgram();

th3ActualAccel = derivative(derivative(pc104.pos3))/dt/dt;

plot(t,th3ActualAccel);