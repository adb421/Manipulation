function cost = optimizeMotorParamsCost(x, pos1, pos2, pos3, currentControl)
%Pull out the parameters from the x vector
Parameters;
% km1 = x(1); km2 = x(2); km3 = x(3);
% params.J14 = x(4);
% params.J11 = x(5);
% params.J8  = x(6);
% params.muS1 = x(7);
% params.muS2 = x(8);
% params.muS3 = x(9);
% params.muD1 = x(10);
% params.muD2 = x(11);
% params.muD3 = x(12);
km1 = x(1); km2 = x(2); km3 = 0;%km3 = 2.10;
% km1 = 2.92; km2 = 2.46; km3 = 2.10;
params.J14 = x(3); params.J11 = x(4);
params.muS1 = x(5); params.muS2 = x(6);
params.muD1 = x(7); params.muD2 = x(8);

initialState = [pos1(1), pos2(1), pos3(1), 0, 0,0];

control = @(t) currentControl(t).*[km1; km2; km3];

dt = 0.001;
tEnd = 3;

[~, states] = ...
    eulerIntegrator(@(t,state)armODE(t,state,params,control),[0,tEnd],dt,initialState);
states(isnan(states)) = 1e12;
cost = sum((states(1,:) - pos1).^2 + (states(2,:) - pos2).^2);