clear
params = ParametersFunction();
dt = 0.001;
T = 5;
K = lqr(params.A,params.B,params.Q,params.R);
xDes = [0;0;-0.25;0;-0.1;0;pi/2-atan(params.wo/params.lo);0];
%   (xm   xmd ym ymd) thm thmd xo,xod,yo,yod,tho,thod
startingTheta = pi/2-atan(params.wo/params.lo)+0.42;
x0 = [-0.2;0; 0; 0;  0;   0;-0.2;0 ; ...
    0+params.wm + params.wo*cos(startingTheta) + ...
    params.lo*sin(startingTheta); 0;startingTheta;0];
params.s1 = params.lm;
t = 0:dt:T;
state = zeros(12,length(t));
u = zeros(3,length(t));
state(:,1) = x0;
for i = 1:length(t)-1
    error = xDes - state(5:end,i);
    u(:,i) = K*error;
    [T, Y] = ode45(@(t,y)onePointRolling(t,state(:,i),u(:,i),params), ...
        [t(i) t(i+1)], state(:,i));
    state(:,i+1) = Y(end,:)';
end
Ff = zeros(size(length(T)));
Fn = zeros(size(length(T)));
for i = 1:length(t)
    [dx, Ff_mTemp, Fn_mTemp] = onePointRolling(t(i),state(:,i), ...
        u(:,i),params);
    Ff(i) = Ff_mTemp*params.mo;
    Fn(i) = Fn_mTemp*params.mo;
end
% [currents, jointAccels] = jointCurrentFromCartAccel(u,Y,params);
figure;
plot(t,state(1,:),'-r',t,state(7,:),'-b',t,xDes(3,:),'-g');
xlabel('Time (s)'); ylabel('X position (m)');
title('Red - manipulator, Blue - object, Green - desired');
figure;
plot(t,state(3,:),'-r',t,state(9,:),'-b',t,xDes(5,:),'-g');
xlabel('Time (s)'); ylabel('Y position (m)');
title('Red - manipulator, Blue - object, Green - desired');
figure;
plot(t,state(5,:),'-r',t,state(11,:),'-b',t,xDes(1,:),'--g',t,xDes(7,:),'-g');
xlabel('Time (s)'); ylabel('Orientation (rad)');
title('Red - manipulator, Blue - object, Green - desired (-- is manip)');
figure
plot(t,u(1,:),'-r',t,u(2,:),'-b');
xlabel('Time (s)'); ylabel('Manipulator Accelerations m/s^2');
title('Red - X accel, Blue - Y accel');
figure
plot(t,u(3,:));
xlabel('Time (s)'); ylabel('Manipulator Accelerations rad/s^2');
title('Angular Accel');
% figure
% plot(t,currents(1,:),'-r',t,5.4*ones(size(t)),'--r', ...
%     t,currents(2,:),'-b',t,2.1*ones(size(t)),'--b', ...
%     t,currents(3,:),'-g',t,1.6*ones(size(t)),'--g');
% title('Red - RH14, Blue - RH11, Green - RH8');
% xlabel('Time (s)');
% ylabel('Current (Amps)');
% 
% figure
% subplot(3,1,1)
% plot(t,jointAccels(1,:),'-r'); title('RH14'); xlabel('Time (s)'); 
% ylabel('rad/s^2');
% subplot(3,1,2)
% plot(t,jointAccels(2,:),'-r'); title('RH11'); xlabel('Time (s)'); 
% ylabel('rad/s^2');
% subplot(3,1,3)
% plot(t,jointAccels(3,:),'-r'); title('RH8'); xlabel('Time (s)'); 
% ylabel('rad/s^2');

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