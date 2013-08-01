% Adam Barber
% 6/15/2012
% Trying to optimize to find system parameters for motor friction and
% models

%Things we want to optimize:
% km1, km2, km3, J14, J11, J8, muS1, muS2, muS3, muD1, muD2, muD3
%Initialize from data sheet
% x0 = [2.92; 2.46; 2.10; ...
%       0.0216; 0.011; 0.0037; ...
%       0; 0; 0; ...
%       0.035*30/pi; 0.018*30/pi; 0.0097*30/pi];
% x0 = [2.92; 2.46; ...
%       0.0216; 0.011;...
%       0; 0;...
%       0.035*30/pi; 0.018*30/pi];
tspan = 0:dt:T;

x0 = [2.3333; 2.3333; ...
      0.0635; 0.0176; ...
      0.2962; 0.2962/2.0; ...
      0.4260; 0.4260/2.0];

currentControl = @(t)[interp1(tspan,curr1,t); interp1(tspan,curr2,t);0];

minFxn = @(x) optimizeMotorParamsCost(x,pos1,pos2,pos3,currentControl);
lb = ones(size(x0)).*[2; 2; 1e-5; 1e-5; 1e-3; 1e-3; 1e-3; 1e-3];
ub = ones(size(x0)).*[4; 4; 1; 0.1; 2; 2; 2; 2];

opts = optimset('Algorithm', 'active-set', 'Display', 'iter', ...
    'MaxFunEvals',1200, 'TolFun', 1e-12, 'TolX', 1e-15);

% xSol = fmincon(minFxn, x0, [],[],[],[],lb,ub, [], opts);
xSol = fmincon(minFxn, xSolGood1, [],[],[],[],lb,ub, [], opts);

% xSol = fminsearch(minFxn, x0);