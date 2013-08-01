%Makes plots from the handle structure saved from the Arm3DoF Gui.

%Pull out some useful data

t = handles.t; %time vector
dt = handles.dt; %time step
%desired planar/joint positions
th1Des = handles.th1Des;
th2Des = handles.th2Des;
th3Des = handles.th3Des;
xDes = handles.xDes;
yDes = handles.yDes;
thDes = handles.thDes;
%actual planar/joint positions
th1Act = handles.th1Act;
th2Act = handles.th2Act;
th3Act = handles.th3Act;
%Control vals
control1 = handles.control1;
control2 = handles.control2;
control3 = handles.control3;
saturated1 = (control1 >= 5.4 | control1 <= -5.4)*5.4;
saturated1(~saturated1) = NaN;
saturated2 = (control2 >= 2.1 | control2 <= -2.1)*2.1;
saturated2(~saturated2) = NaN;
saturated3 = (control3 >= 1.6 | control3 <= -1.6)*1.6;
saturated3(~saturated3) = NaN;

%Calculate joint vels
th1DesVel = derivative(th1Des)/dt;
th2DesVel = derivative(th2Des)/dt;
th3DesVel = derivative(th3Des)/dt;
th1ActVel = derivative(th1Act)/dt;
th2ActVel = derivative(th2Act)/dt;
th3ActVel = derivative(th3Act)/dt;
%Calculate joint accels
th1DesAcc = derivative(th1DesVel)/dt;
th2DesAcc = derivative(th2DesVel)/dt;
th3DesAcc = derivative(th3DesVel)/dt;
th1ActAcc = derivative(th1ActVel)/dt;
th2ActAcc = derivative(th2ActVel)/dt;
th3ActAcc = derivative(th3ActVel)/dt;

%Calculate x-y-th positions
[xDes yDes thDes] = ...
    forwardKin3DoF(th1Des, th2Des, th3Des, handles.params);
[xAct yAct thAct] = ...
    forwardKin3DoF(th1Act, th2Act, th3Act, handles.params);
%Calculate x-y-th vels
xDesVel = derivative(xDes)/dt;
yDesVel = derivative(yDes)/dt;
thDesVel = derivative(thDes)/dt;
xActVel = derivative(xAct)/dt;
yActVel = derivative(yAct)/dt;
thActVel = derivative(thAct)/dt;
%Calculate x-y-th accel
xDesAcc = derivative(xDesVel)/dt;
yDesAcc = derivative(yDesVel)/dt;
thDesAcc = derivative(thDesVel)/dt;
xActAcc = derivative(xActVel)/dt;
yActAcc = derivative(yActVel)/dt;
thActAcc = derivative(thActVel)/dt;

figure;
plot(t,xDes,'-b',t,xAct,'-r');
title('x position');
xlabel('Time (s)'); ylabel('x (m)');
ylim([-0.6 0.6]);

figure;
plot(t,yDes,'-b',t,yAct,'-r');
title('y position');
xlabel('Time (s)'); ylabel('y (m)');
ylim([-0.6 0.6]);

figure;
plot(t,thDes,'-b',t,thAct,'-r');
title('Theta position in world frame');
xlabel('Time (s)'); ylabel('\theta (rad)');

figure;
plot(t,xDesVel,'-b',t,xActVel,'-r');
title('x velocity');
xlabel('Time (s)'); ylabel('Velocity (m/s)');

figure;
plot(t,yDesVel,'-b',t,yActVel,'-r');
title('y velocity');
xlabel('Time (s)'); ylabel('Velocity (m/s)');

figure;
plot(t,thDesVel,'-b',t,thActVel,'-r');
title('Theta velocity in world frame');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');

figure;
plot(t,xDesAcc,'-b',t,xActAcc,'-r');
title('x acceleration');
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');

figure;
plot(t,yDesAcc,'-b',t,yActAcc,'-r');
title('y acceleration');
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');

figure;
plot(t,thDesAcc,'-b',t,thActAcc,'-r');
title('Theta acceleration in world frame');
xlabel('Time (s)'); ylabel('Angular Acceleration (rad/s^2)');

figure;
plot(t,control1,'-r');
title('Control RH14 Joint 1');
xlabel('Time (s)'); ylabel('Current (A)');
ylim([-5.4 5.4])

figure;
plot(t,control2,'-r');
title('Control RH11 Joint 2');
xlabel('Time (s)'); ylabel('Current (A)');
ylim([-2.1 2.1])

figure;
plot(t,control3,'-r');
title('Control RH8 Joint 3');
xlabel('Time (s)'); ylabel('Current (A)');
ylim([-1.6 1.6])

figure;
plot(t,th1Des,'-b',t,th1Act,'-r');
title('Joint 1 position');
xlabel('Time (s)'); ylabel('Angular position (rad)');
ylim([-handles.params.theta1_max handles.params.theta1_max]);

figure;
plot(t,th2Des,'-b',t,th2Act,'-r');
title('Joint 2 position');
xlabel('Time (s)'); ylabel('Angular position (rad)');
ylim([-handles.params.theta2_max handles.params.theta2_max]);

figure;
plot(t,th3Des,'-b',t,th3Act,'-r');
title('Joint 3 position');
xlabel('Time (s)'); ylabel('Angular position (rad)');

figure;
plot(t,th1DesVel,'-b',t,th1ActVel,'-r',t,saturated1,'go');
title('Joint 1 velocity');
xlabel('Time (s)'); ylabel('Angular velocity (rad/s)');

figure;
plot(t,th2DesVel,'-b',t,th2ActVel,'-r',t,saturated2,'go');
title('Joint 2 velocity');
xlabel('Time (s)'); ylabel('Angular velocity (rad/s)');

figure;
plot(t,th3DesVel,'-b',t,th3ActVel,'-r',t,saturated3,'go');
title('Joint 3 velocity');
xlabel('Time (s)'); ylabel('Angular velocity (rad/s)');

figure;
plot(t,th1DesAcc,'-b',t,th1ActAcc,'-r',t,saturated1,'go');
title('Joint 1 acceleration');
xlabel('Time (s)'); ylabel('Angular acceleration (rad/s^2)');

figure;
plot(t,th2DesAcc,'-b',t,th2ActAcc,'-r',t,saturated2,'go');
title('Joint 2 acceleration');
xlabel('Time (s)'); ylabel('Angular acceleration (rad/s^2)');

figure;
plot(t,th3DesAcc,'-b',t,th3ActAcc,'-r',t,saturated3,'go');
title('Joint 3 acceleration');
xlabel('Time (s)'); ylabel('Angular acceleration (rad/s^2)');

figure;
plot(xDes,yDes,'-b',xAct,yAct,'-r');
axis('equal');
title('End point position')
xlabel('x (m)'); ylabel('y (m)');

figure;
plot(t,handles.loopTimes, t, ones(size(t)));
title('Control loop times');
xlabel('Time (s)'); ylabel('Loop times (ms)');
