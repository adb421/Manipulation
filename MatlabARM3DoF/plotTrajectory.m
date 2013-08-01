% Function sends a trajectory over to the pic, recieves the trajectory, and
% plots the joint errors as well as the manipulator position in the plane

function plotTrajectory(t, th1Des, th2Des, th3Des, ...
    th1, th2, th3, control1, control2, control3, params)

[x y] = forwardKin3DoF(th1, th2, th3, params);
[xDes,yDes] = forwardKin3DoF(th1Des,th2Des,th3Des,params);

th = th1Des+th2Des+th3Des;
thDes = th1+th2+th3;

dt = derivative(t);

xVel = derivative(x)./dt;
yVel = derivative(y)./dt;
thVel = derivative(th)./dt;
xVelDes = derivative(xDes)./dt;
yVelDes = derivative(yDes)./dt;
thVelDes = derivative(thDes)./dt;

xAcc = derivative(xVel)./dt;
yAcc = derivative(yVel)./dt;
thAcc = derivative(thVel)./dt;
xAccDes = derivative(xVelDes)./dt;
yAccDes = derivative(yVelDes)./dt;
thAccDes = derivative(thVelDes)./dt;

th1Vel = derivative(th1)./dt;
th2Vel = derivative(th2)./dt;
th3Vel = derivative(th3)./dt;

th1VelDes = derivative(th1Des)./dt;
th2VelDes = derivative(th2Des)./dt;
th3VelDes = derivative(th3Des)./dt;

th1Acc = derivative(th1Vel)./dt;
th2Acc = derivative(th2Vel)./dt;
th3Acc = derivative(th3Vel)./dt;

th1AccDes = derivative(th1VelDes)./dt;
th2AccDes = derivative(th2VelDes)./dt;
th3AccDes = derivative(th3VelDes)./dt;

figure
plot(t,xDes,'-b','DisplayName','Commanded x position');
hold on
plot(t,x,'--b','DisplayName','Actual x position');
plot(t,yDes,'-r','DisplayName','Commanded y position');
plot(t,y,'--r','DisplayName','Actual y position');
xlabel('Time (s)'); ylabel('End point position (m)');
title('Trajectory tracking of x-y position');
legend('show')
hold off

figure
plot(xDes,yDes,'-b','DisplayName','Commanded');
hold on
plot(x,y,'-r','DisplayName','Actual');
xlabel('X-position (m)'); ylabel('Y-position (m)');
title('Position of end point in x-y space');
axis equal
legend('show')
hold off

figure
plot(t,thDes,'-g','DisplayName','Commanded orientation');
hold on
plot(t,th,'--g','DisplayName','Actual orientation');
ylabel('Angle (rad)'); xlabel('Time (s)');
title('End point orientation in world frame');
legend('show')
hold off

figure
plot(t,xVelDes,'-b','DisplayName','Commanded x velocity');
hold on
plot(t,xVel,'--b','DisplayName','Actual x velocity');
plot(t,yVelDes,'-r','DisplayName','Commanded y velocity');
plot(t,yVel,'--r','DisplayName','Actual y velocity');
xlabel('Time(s)'); ylabel('Velocity (m/s)');
title('Velocity tracking of x and y');
legend('show')
hold off

figure
plot(t,thVelDes,'-g','DisplayName','Commanded theta velocity');
hold on
plot(t,thVel,'--g','DisplayName','Actual theta velocity');
xlabel('Time(s)'); ylabel('Angular Velocity (rad/s)');
title('Velocity tracking of end point orientation in world frame');
legend('show')
hold off

figure
plot(t,xAccDes,'-b','DisplayName','Commanded x acceleration');
hold on
plot(t,xAcc,'--b','DisplayName','Actual x acceleration');
plot(t,yAccDes,'-r','DisplayName','Commanded y acceleration');
plot(t,yAcc,'--r','DisplayName','Actual y acceleration');
xlabel('Time(s)'); ylabel('Acceleration (m/s^2)');
title('Acceleration tracking of x and y');
legend('show')
hold off

figure
plot(t,thAccDes,'-g','DisplayName','Commanded theta acceleration');
hold on
plot(t,thAcc,'--g','DisplayName','Actual theta acceleration');
xlabel('Time(s)'); ylabel('Angular Acceleration (rad/s^2)');
title('Acceleration tracking of end point orientation in world frame');
legend('show')
hold off

figure
plot(t,th1Des,'-b','DisplayName','Commanded joint 1 position');
hold on
plot(t,th2Des,'-r','DisplayName','Commanded joint 2 position');
plot(t,th3Des,'-g','DisplayName','Commanded joint 3 position');
plot(t,th1,'--b','DisplayName','Actual joint 1 position');
plot(t,th2,'--r','DisplayName','Actual joint 2 position');
plot(t,th3,'--g','DisplayName','Actual joint 3 position');
xlabel('Time(s)'); ylabel('Angular position (rad)');
title('Joint level trajectory tracking');
legend('show')
hold off

figure
plot(t,th1VelDes,'-b','DisplayName','Commanded joint 1 velocity');
hold on
plot(t,th2VelDes,'-r','DisplayName','Commanded joint 2 velocity');
plot(t,th3VelDes,'-g','DisplayName','Commanded joint 3 velocity');
plot(t,th1Vel,'--b','DisplayName','Actual joint 1 velocity');
plot(t,th2Vel,'--r','DisplayName','Actual joint 2 velocity');
plot(t,th3Vel,'--g','DisplayName','Actual joint 3 velocity');
xlabel('Time(s)'); ylabel('Angular velocity (rad/s)');
title('Joint level velocity tracking');
legend('show')
hold off

figure
plot(t,th1AccDes,'-b','DisplayName','Commanded joint 1 acceleration');
hold on
plot(t,th2AccDes,'-r','DisplayName','Commanded joint 2 acceleration');
plot(t,th3AccDes,'-g','DisplayName','Commanded joint 3 acceleration');
plot(t,th1Acc,'--b','DisplayName','Actual joint 1 acceleration');
plot(t,th2Acc,'--r','DisplayName','Actual joint 2 acceleration');
plot(t,th3Acc,'--g','DisplayName','Actual joint 3 acceleration');
xlabel('Time(s)'); ylabel('Angular acceleration (rad/s^2)');
title('Joint level acceleration tracking');
legend('show')
hold off

figure
plot(t, control1, '-b','DisplayName','Control joint 1');
hold on
plot(t, control2, '-r','DisplayName','Control joint 2');
plot(t, control3, '-g','DisplayName','Control joint 3');
xlabel('Time (s)'); ylabel('PWM Value');
ylim([0 1000]);
title('Control effort');
legend('show')
hold off

end