function params = ParametersFunction()

%Parameters for the arm
params.L1 = 0.193675; params.L2 = 0.19685;% params.L3 = 0.195; % m
params.mm2 = 0.489; params.mm3 = 0.35; params.m1 = 0.387; %kg
params.m2 = 0.193; %params.m3 = 0.0405; 
params.I1 = params.m1*params.L1^2/12.0;
params.I2 = params.m2*params.L2^2/12.0;

params.m3 = 0.0276+0.1146;
params.I3 = .0276*(.205^2 + 0.026^2)/12.0 + ...
    0.1146*(.09^2 + 0.0255^2)/12.0;

params.J14 = 0.0216;
params.J11 = 0.043;
params.J8  = 0.0037;
params.muS1 = 1.2556;
params.muS2 = 1.522;
params.muS3 = 0.504;
params.muD1 = .035*30/pi;
params.muD2 = .017*30/pi;
params.muD3 = .0097*30/pi;

params.km1 = 2.92;
params.km2 = 4.91;
params.km3 = 2.10;


params.maxCurrRH14 = 5.4;
params.maxCurrRH11 = 2.1;
params.maxCurrRH8 = 1.6;

%Inner loop controls
params.k1RH14 = 0.0001 / 2; %Good for RH14
params.k2RH14 = 1.1/ 2; %Pretty damn good for RH14
params.k1RH11 = 0.1/ 2; %Working on RH11 now
params.k2RH11 = 0.125/ 2;
params.k1RH8 = 0.0000001/ 2;
params.k2RH8 = 0.05/ 2;

%Joint control
% params.kp = [650 750 1100]; %For joint control
% params.kd = [75 65 75]; %For joint control, good for joint 1
% params.ki = [0 0 0]; %Joint control
% %Manipulator Control
% params.kp = [1750.0 1750.0 1200.0];
% params.kd = [90.0 90.0 150.0];
%Change these, see if frequency response changes
params.kp = [150.0 150.0 500.0];
params.kd = [10.0 10.0 50.0];

% params.kp = [200.0 200.0 500.0]; %Using these 5/29
% params.kd = [20.0 20.0 50.0];
% params.kp = [265 265 265];
% params.kd = [100.0 100.0 125.0];
% params.kp = [1000.0 1000.0 1000.0]; %For manipulator control only
% params.kd = [40.0 40.0 60.0]; %For manipulator control
% params.kd = [125.0 125.0 125.0];
% params.kp = [265.0 265.0 265.0];
% params.kd = [100.0 100.0 125.0];
% params.ki = [0.0001 0.0001 0.0001]; %For manipulator control

%Dynamic grasp?
% params.kp = [200.0 150.0 200.0]; 
% params.kd = [1.2 1.2 1.2]; 
% params.ki = [0 0 0]; 
% params.kp = [45.0 45.0 45.0]; %Using these for "home" positioning
% params.kd = [25.0 25.0 20.0]; %Using these for "home" position
% params.kp = [265.0 265.0 265.0];
% params.kd = [50.0 55.0 35.0]; %This sort of works!
% params.kd = [10, 10, 10];
% params.kd = [50.0 55.0 20.0];
params.ki = [0 0 0];
% params.kpcurr = [300.0 10.0 10.0];
% params.kdcurr = [5.0 0.0 0.0];

params.theta1_max = 107.57 * pi/180; %Max angle in radians
params.theta2_max = 139.64 * pi/180; %Max angle in radians

params.objHomeX = -0.2;
params.objHomeY = -0.15;
params.objHomeTh = 0.0;

% %Plastic rectangular object
% params.lo = 0.0445;
% params.wo = 0.0285;
% params.mo = 0.088;
% Wooden rectangular object
params.lo = 0.051;
params.wo = 0.045;
params.mo = 0.07545;
params.mu = 1.5;
params.Jo = 1/3*params.mo*(params.lo^2 + params.wo^2);

params.wm = 0.0255;
params.g = 9.81*sin(0.405);
params.lm = 0.09;
params.s1 = params.lm;
params.lc = sqrt(params.lo^2+params.wo^2);
params.objAngle = atan(params.wo/params.lo);
params.mm = 0.0276+0.1146;
params.Jm = 0.0276*(0.205^2 + 0.026^2)/12 + 0.1146*(0.18^2 + 0.052^2)/12;

A = [0 1 0 0 0 0 0 0;...
     0 0 0 0 0 0 0 0;...
     0 -params.wm 0 1 0 0 0 -params.lc;...
     0 0 0 0 0 0 -12*params.g/13  0;...
     0 0 0 0 0 1 0 0;...
     0 0 0 0 0 0 0 0;...
     0 0 0 0 0 0 0 1;...
     0 0 0 0 0 0 12*params.g/(13*params.lc) 0];

B = [0 0 0;...
     0 0 1;...
     0 0 0;...
     1/13 0 -params.wm/13;...
     0 0 0;...
     0 1 0;...
     0 0 0;...
     12/(13*params.lc) 0 -12*params.wm/(13*params.lc)];

params.A = A;
params.B = B;
params.Q = eye(8); 
params.R = eye(3);
%Q should be diag, elements:
%1 -thm, 2-thmd, 3-xo,4-xod,5-yo,6-yod,7-tho,8-thod
%These lead to "limit cycle": 5000/750/500/1
% params.Q(1,1) = 5000; %thm
% params.Q(3,3) = 750; %xo
% params.Q(5,5) = 500; %yo
% params.Q(7,7) = 1; %tho
%8/7, these are good, x is drifting again, maybe drop down tho costs?
% params.Q(1,1) = 10000; %thm %Pretty good, 8/15
% params.Q(2,2) = 100; %thmd
% params.Q(3,3) = 4000; %xo
% params.Q(4,4) = 2000; %xod
% params.Q(5,5) = 2000; %yo
% params.Q(6,6) = 500; %yod
% params.Q(7,7) = 1; %tho
% params.Q(8,8) = 1; %thod
% %Start messing with trajectory stuff for Trajectory, 8/16
% params.Q(1,1) = 100000; 
% params.Q(2,2) = 1; %thmd
% params.Q(3,3) = 8000; %xo
% params.Q(4,4) = 2000; %xod
% params.Q(5,5) = 2000; %yo
% params.Q(6,6) = 500; %yod
% params.Q(7,7) = 1; %tho
% params.Q(8,8) = 1; %thod
%Screwing with 1-pt roll "hold" again.
params.Q(1,1) = 10000; %thm %Pretty good, 8/15
params.Q(2,2) = 100; %thmd
params.Q(3,3) = 4000; %xo
params.Q(4,4) = 2000; %xod
params.Q(5,5) = 3000; %yo
params.Q(6,6) = 500; %yod
params.Q(7,7) = 1; %tho
params.Q(8,8) = 10; %thod
% params.Q(1,1) = 100000; %thm 
% params.Q(2,2) = 1000; %thmd
% params.Q(3,3) = 5500; %xo
% params.Q(4,4) = 1500;
% % params.Q(4,4) = 5000; %xod
% params.Q(5,5) = 1000; %yo
% params.Q(6,6) = 1000; %yod
% params.Q(7,7) = 1; %tho
% params.Q(8,8) = 1; %thod


% Try applying LQR to all states
      %xm xmd ym ymd thm thmd tho thod
Anew = [0 1 0 0 0 0 0 0; ...
        0 0 0 0 0 0 0 0; ...
        0 0 0 1 0 0 0 0; ...
        0 0 0 0 0 0 0 0; ...
        0 0 0 0 0 1 0 0; ...
        0 0 0 0 0 0 0 0; ...
        0 0 0 0 0 0 0 1; ...
        0 0 0 0 0 0 12*params.g/(13*params.lc) 0];

% Anew = [0 1 0 0 0 0 ; ...
%         0 0 0 0 0 0 ; ...
%         0 0 0 1 0 0 ; ...
%         0 0 0 0 0 0 ; ...
%         0 0 0 0 0 1 ; ...
%         0 0 0 0 0 0];
        
Bnew = [0 0 0; ...
        1 0 0; ...
        0 0 0; ...
        0 1 0; ...
        0 0 0; ...
        0 0 1; ...
        0 0 0; ...
        12/(13*params.lc) 0 -12*params.wm/(13*params.lc)];

Qnew = eye(8);
Qnew(1,1) = 750; %xm
Qnew(3,3) = 250; %ym
Qnew(5,5) = 20000; %thm
Qnew(7,7) = 1; %tho

params.Anew = Anew;
params.Bnew = Bnew;
params.Qnew = Qnew;
params.Rnew = params.R;


params.NO_CONTROL         0; %apply no torques
params.GO_HOME_JOINTS     1; %go to a specified joint home position
params.TRAJ_IS_CURRENT    2; %trajectories given are desired motor currents
params.FEEDFORWARD_JOINTS 3; %Feed forward only trajectory control, given as joint angle trajs
params.PID_TRAJ_JOINTS    4; %PID trajectory control, given as joint angle trajs
params.FF_PID_TRAJ_JOINTS 5; %Same as above, but with feed forward added
params.DYNAMIC_GRASP_TRAJ 6; %Follow a trajectory assuming dynamic grasp, trajs are object pos
params.DYNAMIC_GRASP_POS  7; %Go to a specified object pos, assume dyn grasp
params.FF_PID_TRAJ_MANIP  8; %Follow a trajectory, trajs are manipulator pos (x,y,th)
params.PID_MANIP_POS      9; %Go to a specified manipulator pos aka home position
params.ONE_POINT_ROLL_BALANCE 10; % Balance one point rolling, stabilize a point
params.NEW_ONE_POINT_ROLL_BALANCE 11; %Balance one point rolling, LQR is on manipulator config instead of object
params.ONE_POINT_ROLL_TRAJ 12; %Balance one point rolling, LQR trajectory tracking
end