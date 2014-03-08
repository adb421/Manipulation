%LQR sim with all 12 states
clear
params = ParametersFunction();
dt = 0.001;
T = 5;
K = lqr(params.Anew, params.Bnew, params.Qnew, params.Rnew);
xDes = [-0.2; 0; ... %xm, xmdot
    -.1-params.lc; 0; ... %ym ymdot
    0; 0; ... %thm, thmdot
    pi/2-params.objAngle; 0]; %tho, thodot
startingAngle = pi/2 - params.objAngle + 0.35;
x0 = [-0.2; 0; ... %xm xmdot
    0; 0; ... %ym ymdot
    0; 0; ... %thm thmdot
    -0.2 + params.lc*cos(startingAngle+params.objAngle); 0; ... %xo xodot
    params.wm + params.lc*sin(startingAngle+params.objAngle); 0; ... %yo yodot
    startingAngle; 0]; %tho thodot
params.s1 = params.lm;
t = 0:dt:T;
state = zeros(12,length(t));
u = zeros(3,length(t));
error = zeros(8,length(t));
state(:,1) = x0;
desIndices = [1 2 3 4 5 6 11 12];
for i = 1:length(t) - 1
    error(:,i) = xDes - state(desIndices,i);
    error(7,i) = 0; error(8,i) = 0;
    u(:,i) = K*error(:,i);
    [T, Y] = ode45(@(t,y)onePointRolling(t,state(:,i),u(:,i),params), ...
        [t(i) t(i+1)], state(:,i));
    state(:,i+1) = Y(end,:)';
end
Ff = zeros(size(length(T)));
Fn = zeros(size(length(T)));
tBreak = -1;
tFriction = -1;
for i = 1:length(t)
    [dx, Ff_mTemp, Fn_mTemp] = onePointRolling(t(i),state(:,i), ...
        u(:,i),params);
    Ff(i) = Ff_mTemp*params.mo;
    Fn(i) = Fn_mTemp*params.mo;
    if(abs(Ff(i)) >= abs(Fn(i)) && tFriction == -1)
        tFriction = t(i);
    end
    if(Fn(i) <= 0 && tBreak == -1)
        tBreak = t(i);
    end
end        
if(tBreak ~= -1)
    disp(['Contact broken at time t = ' num2str(tBreak) 's']);
end
if(tFriction ~= -1)
    disp(['Friction cone violated at time t = ' num2str(tFriction) 's']);
end
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