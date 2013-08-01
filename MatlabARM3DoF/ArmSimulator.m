%Arm simulation

%Adam Barber
%3/7/2012

%State of the robot is [q1 q2 q3 q1dot q2dot q3dot]'
% initialState = [pi/2;0;0;0;0;0];
% initialState = [-1.0428; 1.0616; 0; 0;0;0];
initialState = [pos1(1); pos2(1); pos3(1); 0; 0; 0];

%Time of integration
dt = 0.001; %seconds
tEnd = 3; %seconds

%Parameters
%L1(m)1 - L2(m)2 - L3(m)3 - mm2(kg)4 - mm3(kg)5 - m1(kg)6 - m2(kg)7 - 
%m3(kg)8 - I1(kg-m^2)9 - I2(kg-m^2)10 - I3(kg-m^2)11 - J14(kg-m^2)12 -
%J11(kg-m^2)13 - J8(kg-m^2)14 - muS1(Nm)15 - muS2(Nm)16 - muS3(Nm)17 -
%muD1(Nm-s)18 - muD2(Nm-s)19 - muD3(Nm-s)20
Parameters;
params.km1 = xSol(1);
params.km2 = xSol(2);
params.km3 = 2.10;
params.J14 = xSol(3); params.J11 = xSol(4); params.J8 = 0.0037;
params.muS1 = xSol(5); params.muS2 = xSol(6); params.muS3 = 0;
params.muD1 = xSol(7); params.muD2 = xSol(8); params.muD3 = 0.0097*30/pi;

control = @(t)[interp1(tspan,curr1*params.km1,t);interp1(tspan,curr2*params.km2,t);0];

%[times states] = ode45(@(t,state)armODE(t,state,params),0:dt:tEnd,initialState);
[times states] = eulerIntegrator(@(t,state)armODE(t,state,params, control),[0 tEnd], dt, initialState);

figure;
hold on
plot(times, states(1,:), '--b');
plot(times, pos1, '-b');
plot(times, states(2,:), '--g');
plot(times, pos2, '-g');
% plot(times, states(3,:), '--r');
% figure;
% hold on
% plot(times, states(4,:), '-b');
% plot(times, states(5,:), '-g');
% plot(times, states(6,:), '-r');

F = animateArm(times,states,params,dt);