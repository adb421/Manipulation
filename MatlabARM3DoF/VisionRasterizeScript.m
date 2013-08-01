%Do a slow moving trajectory to try and find how smooth the vision system is.

%Create the trajectory;
dt = 0.001;
T1 = 5; T2 = T1+2; T3 = T2+5; T4 = T3 + 2;
T5 = T4 + 5; T6 = T5+2; T7 = T6 + 5;
T8 = T7 + 2; T9 = T8 + 5;
x0 = -0.05;     y0 = -0.3;
x1 = -0.05;     y1 = -0.15;
x2 = -0.1; y2 = -0.15;
x3 = -0.1; y3 = -0.25;
x4 = -0.15; y4 = -0.25;
x5 = -0.15; y5 = -0.15;
x6 = -0.2; y6 = -0.15;
x7 = -0.2; y7 = -0.2;
x8 = -0.25; y8 = -0.2;
x9 = -0.25; y9 = -0.15;
t1 = 0:dt:T1; t2 = T1+dt:dt:T2;
t3 = T2+dt:dt:T3; t4 = T3+dt:dt:T4;
t5 = T4+dt:dt:T5; t6 = T5+dt:dt:T6;
t7 = T6+dt:dt:T7; t8 = T7+dt:dt:T8;
t9 = T8+dt:dt:T9;
t = [t1 t2 t3 t4 t5 t6 t7 t8 t9];
num_pts = length(t);
x01 = linspace(x0,x1,length(t1)); x12 = linspace(x1, x2, length(t2));
x23 = linspace(x2,x3,length(t3)); x34 = linspace(x3, x4, length(t4));
x45 = linspace(x4,x5,length(t5)); x56 = linspace(x5, x6, length(t6));
x67 = linspace(x6,x7,length(t7)); x78 = linspace(x7, x8, length(t8));
x89 = linspace(x8,x9,length(t9));
y01 = linspace(y0,y1,length(t1)); y12 = linspace(y1, y2, length(t2));
y23 = linspace(y2,y3,length(t3)); y34 = linspace(y3, y4, length(t4));
y45 = linspace(y4,y5,length(t5)); y56 = linspace(y5, y6, length(t6));
y67 = linspace(y6,y7,length(t7)); y78 = linspace(y7, y8, length(t8));
y89 = linspace(y8,y9,length(t9));
xTraj = [x01 x12 x23 x34 x45 x56 x67 x78 x89];
yTraj = [y01 y12 y23 y34 y45 y56 y67 y78 y89];
thTraj = zeros(size(xTraj));


%Create the object to communicate with PC104
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();
pc104.sendControlGains();
home = [xTraj(1), yTraj(1), thTraj(1)];
pc104.sendHomePos(home);
disp('unpause to go home');
pause
pc104.goHome();
pc104.allocateTraj(num_pts);
disp('unpause to send traj');
pause
pc104.sendTraj(xTraj,yTraj,thTraj,t);
disp('unpause to go traj');
pause
pc104.goTraj();
pause(T9+dt);
pc104.getTrajData();
pc104.plotTrajData();