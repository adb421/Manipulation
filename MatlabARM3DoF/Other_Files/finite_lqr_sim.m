%Calculates LQR gains and then simulates the one point rolling. Let's see
%what happens


%Set up params struct
params = ParametersFunction();
params.s1 = params.lm;
params.g = 9.81;

% %Generate xd and ud
% T = 10;
% tspan = 0:0.001:T;
% %states are thm, thmd, xo, xod, yo, yod, tho, thod
% %controls are xmdd, ymdd, thmdd
% xf = -0.1; xo = -0.3;
% yf = 0.0; yo = 0.1;
% ax = xo; bx = 0; cx = 3*(xf - xo)/T^2; dx = 2*(xo - xf)/T^3;
% ay = yo; by = 0; cy = 3*(yf - yo)/T^2; dy = 2*(yo - yf)/T^3;
% xd = [zeros(1,length(tspan));
%       zeros(1,length(tspan));
%       polyval([dx cx bx ax], tspan);
%       polyval(polyder([dx cx bx ax]),tspan);
%       polyval([dy cy by ay], tspan);
%       polyval(polyder([dy cy by ay]),tspan);
%       (pi/2 - params.objAngle)*ones(size(tspan));
%       zeros(size(tspan))];
% % xd = [zeros(1,length(tspan));
% %       zeros(1,length(tspan));
% %       -0.2*ones(1,length(tspan));
% %       zeros(1,length(tspan));
% %       -0.1*ones(1,length(tspan));
% %       zeros(1,length(tspan));
% %       (pi/2-params.objAngle)*ones(1,length(tspan));
% %       zeros(1,length(tspan))];
% %Let's try this!
% ud = [polyval(polyder(polyder([dx cx bx ax])),tspan);
%       polyval(polyder(polyder([dy cy by ay])),tspan);
%       zeros(size(tspan))];
T = 0.8;
xo = -0.3; xf = -0.1;
yo = 0.1; yf = 0.0;
%3rd order polynomials that have zero start and end velocity
ax = xo; bx = 0; cx = 3*(xf - xo)/T^2; dx = 2*(xo - xf)/T^3;
ay = yo; by = 0; cy = 3*(yf - yo)/T^2; dy = 2*(yo - yf)/T^3;
xPoly = [dx cx bx ax];
yPoly = [dy cy by ay];
% t1 = 0:0.001:2.999;
t1 = 0:0.001:0.001;
t2 = 0.002:0.001:T;
t3 = T+.001:0.001:T+2.2;
tPoly = t2 - t2(1);
t = [t1 t2 t3];
xd = [ones(1,length(t1))*xo, polyval(xPoly,tPoly), ones(1,length(t3))*xf;
      zeros(1,length(t1)), polyval(polyder(xPoly),tPoly), zeros(1,length(t3));
      ones(1,length(t1))*yo, polyval(yPoly,tPoly), ones(1,length(t3))*yf;
      zeros(1,length(t1)), polyval(polyder(yPoly),tPoly), zeros(1,length(t3));
      ones(1,length(t))*(pi/2-params.objAngle);
      zeros(1,length(t))];
  xd = one_point_roll_min_mu(xd,t,params);

%Try ud = 0? if not, whats happening in the simulator
ud = zeros(3,length(t));
% ud = [polyval(polyder(polyder(xPoly)),tPoly); ...
%     polyval(polyder(polyder(yPoly)),tPoly); ...
%     zeros(1,length(tPoly))];
num_pts = length(t);
tspan = t;
%Flip them for P and R calculations
xdFlip = fliplr(xd);
udFlip = fliplr(ud);

%Pick Q and R
% Q = eye(8);
% R = eye(3);
% Q(1,1) = 10000;
% Q(3,3) = 5;
% Q(4,4) = 1;
% Q(7,7) = 100;
% Q(8,8) = 10;
Q = params.Q;
R = params.R;

% Q = params.Q;
% R = params.R;
%Integrate backwards from T
tspanFlip = fliplr(tspan);

%Calculate P
%figure out trajectories
[Tp, PrFlip] = ode45(@(t,y) ...
    finite_lqr_odefun_Pr(t,y,xdFlip,udFlip,tspanFlip,Q,R,params), ...
    tspanFlip,zeros(72,1));
PrFlip = PrFlip';
Pr = fliplr(PrFlip);

% state = zeros(12,1);
% desIndices = [5 6 7 8 9 10 11 12];
% startingTheta = pi/2-params.objAngle - 0.2;
% thc = params.objAngle;
% xd = [ones(1,length(t1))*xo, polyval(xPoly,tPoly), ones(1,length(t3))*xf;
%       zeros(1,length(t1)), polyval(polyder(xPoly),tPoly), zeros(1,length(t3));
%       ones(1,length(t1))*yo, polyval(yPoly,tPoly), ones(1,length(t3))*yf;
%       zeros(1,length(t1)), polyval(polyder(yPoly),tPoly), zeros(1,length(t3));
%       ones(1,length(t))*(pi/2-params.objAngle);
%       zeros(1,length(t))];
% state(1) = xd(3,1);
% state(2) = 0; state(4) = 0;
% state(3) = xd(5,1) - params.lc - params.wm;
% state(5:end,1) = xd(:,1);
lc = params.lc;
state = [xo; 0; yo - lc - params.wm; 0; 0; 0; ...
    xo + lc*cos(startingTheta+params.objAngle); 0; ...
    yo - lc*(1-sin(startingTheta + params.objAngle)); 0; ...
    startingTheta; 0];

[T, state] = ode45(@(t,y) onePointRolling(t,y,xd,ud,Pr,tspan,R,params), ...
    tspan, state);
    
state = state';

%Calculate U and F
%TODO
u = zeros(3,length(tspan));
Ff = zeros(1,length(tspan));
Fn = zeros(1,length(tspan));
for i = 1:length(tspan)
    [~,uT, FfT, FnT] = ...
        onePointRolling(tspan(i),state(:,i),xd,ud,Pr,tspan,R,params);
    u(:,i) = uT;
    Ff(i) = FfT;
    Fn(i) = FnT;
end
    
t = tspan;
%%
figure;
plot(t,state(1,:),'-r',t,state(7,:),'-b',t,xd(3,:),'-g');
set(gca,'fontsize',20);
xlabel('Time (s)','fontsize',20); ylabel('X position (m)','fontsize',20);
legend('Manipulator','Object','Desired');
figure;
plot(t,state(3,:),'-r',t,state(9,:),'-b',t,xd(5,:),'-g');
xlabel('Time (s)'); ylabel('Y position (m)');
legend('Manipulator','Object','Desired');
figure;
plot(t,state(5,:),'-r',t,state(11,:),'-b',t,xd(1,:),'--g',t,xd(7,:),'-g');
xlabel('Time (s)'); ylabel('Orientation (rad)');
legend('Manipulator','Object','Manipulator Desired','Object Desired');
figure
plot(t,u(1,:),'-r',t,u(2,:),'-b');
xlabel('Time (s)'); ylabel('Manipulator Accelerations m/s^2');
title('Red - X accel, Blue - Y accel');
figure
plot(t,u(3,:));
xlabel('Time (s)'); ylabel('Manipulator Accelerations rad/s^2');
title('Angular Accel');

figure
plot(t,Ff,'-r',t,Fn,'-b', t,abs(Ff),'-g');
title('Friction forces');
xlabel('Time (s)'); ylabel('N');
legend('Friction Force','Normal Force','Absolute Friction Force');

%%
%Animate the everything
figure;
axis off, axis equal
hold on
xlim([-0.5 0])
ylim([-0.5 0.5])
%bottom left, bottom right, top right, top left
xm = state(1,:); ym = state(3,:); thm = state(5,:);
xo = state(7,:); yo = state(9,:); tho = state(11,:);
xverts = [xm(1) - params.lm*cos(thm(1)) + params.wm*sin(thm(1)); ...
          xm(1) + params.lm*cos(thm(1)) + params.wm*sin(thm(1)); ...
          xm(1) + params.lm*cos(thm(1)) - params.wm*sin(thm(1));
          xm(1) - params.lm*cos(thm(1)) - params.wm*sin(thm(1))];
yverts = [ym(1) - params.wm*cos(thm(1)) - params.lm*sin(thm(1)); ...
          ym(1) - params.wm*cos(thm(1)) + params.lm*sin(thm(1)); ...
          ym(1) + params.wm*cos(thm(1)) + params.lm*sin(thm(1));
          ym(1) + params.wm*cos(thm(1)) - params.lm*sin(thm(1))];
manip = patch(xverts,yverts,'r');
xverts = [xo(1) - params.lo*cos(tho(1)) + params.wo*sin(tho(1)); ...
          xo(1) + params.lo*cos(tho(1)) + params.wo*sin(tho(1)); ...
          xo(1) + params.lo*cos(tho(1)) - params.wo*sin(tho(1));
          xo(1) - params.lo*cos(tho(1)) - params.wo*sin(tho(1))];
yverts = [yo(1) - params.wo*cos(tho(1)) - params.lo*sin(tho(1)); ...
          yo(1) - params.wo*cos(tho(1)) + params.lo*sin(tho(1)); ...
          yo(1) + params.wo*cos(tho(1)) + params.lo*sin(tho(1));
          yo(1) + params.wo*cos(tho(1)) - params.lo*sin(tho(1))];
obj = patch(xverts,yverts,'b');
M(length(1:100:length(t))) = struct('cdata',[],'colormap',[]);
j = 1;
for i=1:100:length(t)
    xverts = [xm(i) - params.lm*cos(thm(i)) + params.wm*sin(thm(i)); ...
              xm(i) + params.lm*cos(thm(i)) + params.wm*sin(thm(i)); ...
              xm(i) + params.lm*cos(thm(i)) - params.wm*sin(thm(i));
              xm(i) - params.lm*cos(thm(i)) - params.wm*sin(thm(i))];
    yverts = [ym(i) - params.wm*cos(thm(i)) - params.lm*sin(thm(i)); ...
              ym(i) - params.wm*cos(thm(i)) + params.lm*sin(thm(i)); ...
              ym(i) + params.wm*cos(thm(i)) + params.lm*sin(thm(i));
              ym(i) + params.wm*cos(thm(i)) - params.lm*sin(thm(i))];
    set(manip,'XData',xverts,'YData',yverts);
    xverts = [xo(i) - params.lo*cos(tho(i)) + params.wo*sin(tho(i)); ...
              xo(i) + params.lo*cos(tho(i)) + params.wo*sin(tho(i)); ...
              xo(i) + params.lo*cos(tho(i)) - params.wo*sin(tho(i));
              xo(i) - params.lo*cos(tho(i)) - params.wo*sin(tho(i))];
    yverts = [yo(i) - params.wo*cos(tho(i)) - params.lo*sin(tho(i)); ...
              yo(i) - params.wo*cos(tho(i)) + params.lo*sin(tho(i)); ...
              yo(i) + params.wo*cos(tho(i)) + params.lo*sin(tho(i));
              yo(i) + params.wo*cos(tho(i)) - params.lo*sin(tho(i))];
    set(obj,'XData',xverts,'YData',yverts);
    M(j) = getframe;
    j = j+1;
end
movie(M,1,10)