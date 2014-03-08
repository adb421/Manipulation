%We want to do a simulation of dynamic grasp to then transition to a
%one-point roll control

% Hopefully we'll have a function that determines the trajectory so we don't need to step through the simulation

%Make it a function so we can have everything in one file thanks to matlab
function [t, x, M] = Script_Sim_Dyn_Grasp_To_Balance(xd,td,ud,params)
    lo = params.lo; wo = params.wo;
    lm = params.lm; wm = params.wm;
    s1 = params.s1;
    th0 = xd(5,1);
    x0 = [xd(1,1) - lo*cos(th0) + wo*sin(th0) - s1*cos(th0) + lm*cos(th0) + ...
          wm*sin(th0); %xm
          xd(2,1); %xmd %WRONG
          xd(3,1) - wo*cos(th0) - lo*sin(th0) - s1*sin(th0) - wm*cos(th0) + ...
          lm*sin(th0); %ym
          xd(4,1); %ymd %WRONG
          xd(:,1)]; %qo/qod
    
    %x is the full manipulator/object state
    tRoll = td(end)+0.1:0.1:td(end)+5;
    tdGrasp = td;
    td = [td tRoll];
    xdBal = [zeros(1,length(tRoll)); zeros(1,length(tRoll)); ...
             ones(1,length(tRoll))*xd(1,end); zeros(1,length(tRoll)); ...
             ones(1,length(tRoll))*xd(3,end); zeros(1,length(tRoll)); ...
             ones(1,length(tRoll))*(pi/2-params.objAngle); ...
             zeros(1,length(tRoll))];
    xd = [xd, xdBal];
    udBal = zeros(3,length(tRoll));
    ud = [ud, udBal];
    
    %Do the dynamic grasp simulation
    [tGrasp, xGrasp] = ode45(@(t,y) dynamicGraspODEFun(t,y,params,xd,ud,td), ... 
                             td, x0);
    tGrasp = tGrasp';
    xGrasp = xGrasp';
    for i = 1:length(tGrasp)
        [~, FfT, FnT] = dynamicGraspODEFun(tGrasp(i), ...
            xGrasp(:,i),params,xd,ud,td);
        Ff(i) = FfT;
        Fn(i) = FnT;
    end
    xdBal = [0;0;xd(1,end);0;xd(3,end);0;pi/2-params.objAngle;0];
    udBal = [0;0;0];
    [A,B] = onePointRollingLinear(xdBal, udBal, params);
%     Q = params.Q;
%     Q(1,1) = 10; Q(2,2) = 10;
%     R = params.R;
    Q = eye(8); R = 10*eye(3);
    Q(1,1) = 10000;
    Q(3,3) = 100;
    Q(5,5) = 100;
    Q(7,7) = 0.01;
%    [A, B] = onePointRollingLinear(xdBal(:,1),udBal(:,1),params);
%    Kbal = lqr(A, B, Q, R);
%    kp1 = 5; kp2 = 5; kp3 = 30;
%    kd1 = 1; kd2 = 10; kd3 = 5;
%    K = zeros(3,8,length(tdGrasp));
%    K(1,3,1:length(tdGrasp)) = ones(1,length(tdGrasp))*kp1;
%    K(1,4,1:length(tdGrasp)) = ones(1,length(tdGrasp))*kd1;
%    K(2,5,1:length(tdGrasp)) = ones(1,length(tdGrasp))*kp2;
%    K(2,6,1:length(tdGrasp)) = ones(1,length(tdGrasp))*kd2;
%    K(3,7,1:length(tdGrasp)) = ones(1,length(tdGrasp))*kp3;
%    K(3,8,1:length(tdGrasp)) = ones(1,length(tdGrasp))*kd3;
%    desConMode = 2*ones(1,length(td));
%    for i = 1:length(tRoll)
%        K(:,:,i+length(tdGrasp)) = Kbal;
%        desConMode(i+length(tdGrasp)) = 1;
%    end
%    [t, x] = ode45(@(t,y) overallODE(t,y,params,xd,ud,td,desConMode,K),td,x0);
%    t = t';
%    x = x';
%    for i = 1:length(t)
%        [~, ~, FfT, FnT, sT, num_contactsT] = overallODE(t(i),x(:,i),params,xd,ud,td, ...
%                                               desConMode,K, num_contacts);
%        Ff(i) = FfT; Fn(i) = FnT; s(i) = sT; num_contacts(i) = num_contactsT;
%    end
%     Put this in a roll balance 
    [tRoll, xRoll] = ode45(@(t,y) onePointRollODEFun(t,y,params,xdBal,K), td(end):0.1:td(end)+5, xGrasp(:,end));
    tRoll = tRoll';
    xRoll = xRoll';
%     for i = length(tGrasp)+1:length(tGrasp)+length(tRoll)
    for i = 1:length(tRoll);
        [~, FfT, FnT] = onePointRollODEFun(tRoll(i), ...
            xRoll(:,i),params,xdBal,K);
        Ff(i+length(tGrasp)) = FfT;
        Fn(i+length(tGrasp)) = FnT;
    end
    t = [tGrasp tRoll];
    x = [xGrasp xRoll];
 t = tGrasp; x = xGrasp;
    xd = [xd([1,3,5],:), xdBal([3,5,7])*ones(1,length(tRoll))];
    
    figure;
    subplot(3,1,1);
    title('X position of object CoM')
    plot(t,xd(1,:),t,x(7,:));
    legend('Desired','Actual');
    xlabel('Time (s)'); ylabel('Position (m)');
    subplot(3,1,2);
    title('Y position of object CoM')
    plot(t,xd(2,:),t,x(9,:));
    legend('Desired','Actual');
    xlabel('Time (s)'); ylabel('Position (m)');
    subplot(3,1,3);
    title('Angular position of object CoM')
    plot(t,xd(3,:),t,x(11,:));
    legend('Desired','Actual');
    xlabel('Time (s)'); ylabel('Position (rad)');

    figure;
    title('Contact forces')
    plot(t,Ff,'-r',t,Fn,'-b',t,abs(Ff),'--r');
    xlabel('Time (s)'); ylabel('Force (N)');
    legend('Friction force','Normal force','abs(Friction force)');    
    
    figure;
    axis off, axis equal
    hold on
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    %bottom left, bottom right, top right, top left
    xm = x(1,:); ym = x(3,:); thm = x(5,:);
    xo = x(7,:); yo = x(9,:); tho = x(11,:);
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
    M(length(1:length(t))) = struct('cdata',[],'colormap',[]);
    j = 1;
    for i=1:length(t)
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
end

function [dx, Ff, Fn, s, num_contacts] = overallODE(t,x,params,xd,ud,td,desConMode,K, num_contacts)
if(nargin < 9)
    num_contacts = 0;
end
dx = zeros(12,1);
%pull out useful params first
    g = params.g; mm = params.mm; lm = params.lm; wm = params.wm;
    Jm = params.Jm; mu = params.mu; mo = params.mo; wo = params.wo; lo = params.lo;
    Jo = params.Jo; thc = params.objAngle; lc = params.lc;
    xm = x(1); ym = x(3); thm = x(5); xo = x(7); yo = x(9); tho = x(11);
    epsilon = 1e-5;
    
    %Chcek which points are in contact
    %Calculate number of contact points
    %Calculate bottom left contact point from object
    xc1 = xo - lo*cos(tho) + wo*sin(tho);
    yc1 = yo - wo*cos(tho) - lo*sin(tho);
    %Bottom right contact point
    xc2 = xo + lo*cos(tho) + wo*sin(tho);
    yc2 = yo - wo*cos(tho) + lo*sin(tho);
    %Calculate points of the manipulator
    xm1 = xm - lm*cos(thm) - wm*sin(thm);
    ym1 = ym + wm*cos(thm) - lm*sin(thm);
    xm2 = xm + lm*cos(thm) - wm*sin(thm);
    ym2 = ym + wm*cos(thm) + lm*sin(thm);
    %Calculate the orthogonal distance between contact point one the line
    d = abs(det([xm2-xm1, xm1 - xc1; ym2 - ym1, ym1 - yc1]))/lm;
    s1 = norm([xc1; yc1] - [xm1; ym1]);
    if(d > epsilon)
        if(nargin < 9)
            num_contacts = num_contacts + 1;
        end
        d = abs(det([xm2-xm1,xm1-xc2;ym2-ym1,ym1-yc2]))/lm;
        %Calculate s1
        s1 = norm([xc1; yc1] - [xm1; ym1]);
        if(d > epsilon)
            if(nargin < 9)
                num_contacts = num_contacts+1;
            end
        end
    end
    

    %Velocity is velocity
    dx(1) = x(2); dx(3) = x(4); dx(5) = x(6);
    dx(7) = x(8); dx(9) = x(10); dx(11) = x(12);
    
    %Calculate controls
    xDes = [interp1(td,xd(1,:),t); ...
        interp1(td,xd(2,:),t); ...
        interp1(td,xd(3,:),t); ...
          interp1(td,xd(4,:),t); ...
          interp1(td,xd(5,:),t); ...
          interp1(td,xd(6,:),t); ...
          interp1(td,xd(7,:),t); ...
          interp1(td,xd(8,:),t)];
    uDes = [interp1(td,ud(1,:),t); ...
          interp1(td,ud(2,:),t); ...
          interp1(td,ud(3,:),t)];
    controlMode = round(interp1(td,desConMode,t));

    switch(controlMode)
      case 0 %free flight
        xmdd = 0; ymdd = 0; thmdd = 0;
      case 1 %one point roll
             %INTERP FOR K
        Kuse = [interp1(td,squeeze(K(1,1,:)),t), interp1(td,squeeze(K(1,2,:)),t), ...
                interp1(td,squeeze(K(1,3,:)),t), interp1(td,squeeze(K(1,4,:)),t), ...
                interp1(td,squeeze(K(1,5,:)),t), interp1(td,squeeze(K(1,6,:)),t), ...
                interp1(td,squeeze(K(1,7,:)),t), interp1(td,squeeze(K(1,8,:)),t); ...
               interp1(td,squeeze(K(2,1,:)),t), interp1(td,squeeze(K(2,2,:)),t), ...
                interp1(td,squeeze(K(2,3,:)),t), interp1(td,squeeze(K(2,4,:)),t), ...
                interp1(td,squeeze(K(2,5,:)),t), interp1(td,squeeze(K(2,6,:)),t), ...
                interp1(td,squeeze(K(2,7,:)),t), interp1(td,squeeze(K(2,8,:)),t); ...
               interp1(td,squeeze(K(3,1,:)),t), interp1(td,squeeze(K(3,2,:)),t), ...
                interp1(td,squeeze(K(3,3,:)),t), interp1(td,squeeze(K(3,4,:)),t), ...
                interp1(td,squeeze(K(3,5,:)),t), interp1(td,squeeze(K(3,6,:)),t), ...
                interp1(td,squeeze(K(3,7,:)),t), interp1(td,squeeze(K(3,8,:)),t)];
        u = Kuse*(xd-x(5:end)) + uDes;
        xmdd = u(1); ymdd = u(2); thmdd = u(3);
        Ff = -mm*(cos(thm)*xmdd + sin(thm)*ymdd);
        Fn = mm*(sin(thm)*xmdd - cos(thm)*ymdd);
        thmdd_check = (-fn*(s1-lm) + Ff*wm)/Jm;
        %Check if this is ok?
        if(abs(thmdd_check - thmdd) > epsilon)
            disp('thmdd"s do not match')
        end
      case 2 %Dynamic Grasp mode
             %INTERP FOR K
        Kuse = [interp1(td,squeeze(K(1,1,:)),t), interp1(td,squeeze(K(1,2,:)),t), ...
                interp1(td,squeeze(K(1,3,:)),t), interp1(td,squeeze(K(1,4,:)),t), ...
                interp1(td,squeeze(K(1,5,:)),t), interp1(td,squeeze(K(1,6,:)),t), ...
                interp1(td,squeeze(K(1,7,:)),t), interp1(td,squeeze(K(1,8,:)),t); ...
               interp1(td,squeeze(K(2,1,:)),t), interp1(td,squeeze(K(2,2,:)),t), ...
                interp1(td,squeeze(K(2,3,:)),t), interp1(td,squeeze(K(2,4,:)),t), ...
                interp1(td,squeeze(K(2,5,:)),t), interp1(td,squeeze(K(2,6,:)),t), ...
                interp1(td,squeeze(K(2,7,:)),t), interp1(td,squeeze(K(2,8,:)),t); ...
               interp1(td,squeeze(K(3,1,:)),t), interp1(td,squeeze(K(3,2,:)),t), ...
                interp1(td,squeeze(K(3,3,:)),t), interp1(td,squeeze(K(3,4,:)),t), ...
                interp1(td,squeeze(K(3,5,:)),t), interp1(td,squeeze(K(3,6,:)),t), ...
                interp1(td,squeeze(K(3,7,:)),t), interp1(td,squeeze(K(3,8,:)),t)];
        error = xDes(3:end) - x(7:12);
        u = Kuse(:,3:end)*error + uDes;
        xmdd = u(1) + ((lo + s1 - lm)*cos(x(5)) - (wm + wo)*sin(x(5)))*x(6)^2 ...
               + ((wm + wo)*cos(x(5)) + (lo + s1 - lm)*sin(x(5)))*u(3);
        ymdd = u(2) + ((wm + wo)*cos(x(5)) + (lo + s1 - lm)*sin(x(5)))*x(6)^2 ...
               + ((lm - lo - s1)*cos(x(5)) + (wm + wo)*sin(x(5)))*u(3);
        thmdd = u(3);
        Ff = -mm*(cos(thm)*xmdd + sin(thm)*ymdd);
        Fn = mm*(sin(thm)*xmdd - cos(thm)*ymdd);
        otherwise
    end
    dx(2) = xmdd; dx(4) = ymdd; dx(6) = thmdd;
    switch(num_contacts)
      case 0 %Free flight
        Ff = 0; Fn = 0;
        dx(8) = 0; dx(10) = -g; dx(12) = 0;
        s = NaN;
      case 1 %one point contact
              %Here, we don't have xmdd, ymdd, thmdd given
              %Instead, we'll be given desired Ff, Fn and we'll calculate
              %xmdd and ymdd and thmdd
              %u(1) is Ff, u(2) is Fn
        %Check frictional forces
        s = s1;
        if(Fn < 0)
            %Breaking contact!
            [dx Ff Fn s num_contacts] = overallODE(t,x,params,xd,ud,td,desConMode, ...
                                    K, num_contacts-1);
            return;
        end
        if(abs(Ff) >= mu*Fn)
            Ff = mu*Fn*sign(Ff);
        end
        dx(8) = (Ff*cos(thm) - Fn*sin(thm))/mo;
        dx(10) = (Fn*cos(thm) + Ff*sin(thm) - mg)/mo;
        dx(12) = (Ff*lc*sin(tho + thc - thm) - Fn*lc*cos(tho + thc - thm))/Jo;
      case 2 %Two point contact
        s = (Fn*lm+Ff*wm - Jm*thmdd)/Fn;
        %Check frictional forces
        if(Fn < 0)
            %Breaking contact!
            [dx Ff Fn s num_contacts] = overallODE(t,x,params,xd,ud,td,desConMode, ...
                                    K, num_contacts-1);
            return;
        end
        if(abs(Ff) >= mu*Fn)
            %Friction force exceeded
            Ff = mu*Fn*sign(Ff);
        end
        if(s < s1)
            %wrench applied outside edge
            s = s1;    
        end
        if(s > s1+2*lo)
            %wrench applied outside edge
            s = s1 + 2*lo;
        end
        dx(8) = (Ff*cos(thm) - Fn*sin(thm))/mo;
        dx(10) = (Fn*cos(thm) + Ff*sin(thm) - mo*g)/mo;
        dx(12) = (Ff*wo + Fn*(s - (s1+lo)))/Jo;
    end
end

function [dx Ff Fn] = dynamicGraspODEFun(t,x,params,xd,ud,td)
    %Set up stuff
%     kp1 = 500; kp2 = 500; kp3 = 500;
%     kd1 = 100; kd2 = 100; kd3 = 100;
    kp1 = 5; kp2 = 5; kp3 = 30;
    kd1 = 1; kd2 = 10; kd3 = 5;
    K = [kp1, kd1, 0, 0, 0, 0; ...
        0, 0, kp2, kd2, 0, 0; ...
        0, 0, 0, 0, kp3, kd3];
    mu = params.mu;
    mm = params.mm; Jm = params.Jm;
    wm = params.wm; lm = params.lm;
    wo = params.wo; lo = params.lo;
    s1 = params.s1; g = params.g;
    xDes = [interp1(td,xd(1,:),t); ...
          interp1(td,xd(2,:),t); ...
          interp1(td,xd(3,:),t); ...
          interp1(td,xd(4,:),t); ...
          interp1(td,xd(5,:),t); ...
          interp1(td,xd(6,:),t)];
    uDes = [interp1(td,ud(1,:),t); ...
          interp1(td,ud(2,:),t); ...
          interp1(td,ud(3,:),t)];
    error = xDes - x(7:12);
    u = K*error + uDes;
    %u is in Fx, Fy, tau at the center of mass of the object, transition to
    %contact force and location
%     Ff = u(1)*cos(x(11)) + (u(2) - params.g*params.mo)*sin(x(11));
%     Fn = (u(2) - params.g*params.mo)*cos(x(11)) - u(1)*sin(x(11));
%     s = (u(3) - Ff*wo)/Fn + lo;
    xmdd = u(1) + ((lo + s1 - lm)*cos(x(5)) - (wm + wo)*sin(x(5)))*x(6)^2 ...
        + ((wm + wo)*cos(x(5)) + (lo + s1 - lm)*sin(x(5)))*u(3);
    ymdd = u(2) + ((wm + wo)*cos(x(5)) + (lo + s1 - lm)*sin(x(5)))*x(6)^2 ...
        + ((lm - lo - s1)*cos(x(5)) + (wm + wo)*sin(x(5)))*u(3);
    Ff = mm*(u(1)*cos(x(5)) - (g + u(2))*sin(x(5)));
    Fn = mm*(g + u(2))*cos(x(5)) - mm*u(1)*sin(x(5));
    s = (lm*Fn - Fn*s1 + Jm*u(3) + Ff*wm)/Fn;
    
    if(abs(Ff) > mu*Fn)
        disp(['Friction cone violated at t = ' num2str(t)]);
    end
    if(s < 0 || s > 2*lo)
        disp(['Force line of action doesnt pass through contact at t = ' num2str(t)]);
    end
    if( Fn < 0)
        disp(['Contact broken at t = ' num2str(t)]);
    end
%     xmdd = (Fn*sin(x(5)) - Ff*cos(x(5)))/mm;
%     ymdd = -(Fn*cos(x(5)) + Ff*sin(x(5)))/mm;
%     thmdd = (-Ff*wm + Fn*(s+s1 - lm))/Jm;
    dx = zeros(12,1);
    %Go through dynamics
    %Super simple right now
    dx(1) = x(2);
    dx(3) = x(4);
    dx(5) = x(6);
    dx(7) = x(8);
    dx(9) = x(10);
    dx(11) = x(12);
    dx(2) = xmdd;
    dx(4) = ymdd;
    dx(6) = u(3);
    dx(8) = u(1);
    dx(10) = u(2);
    dx(12) = u(3);
end

function [dx, Ff, Fn] = onePointRollODEFun(t,x,params,xd,K)
    u = K*(xd-x(5:end));
    [dx, Ff_m, Fn_m] = onePointRolling(t,x,u,params);
    Ff = Ff_m*params.mm;
    Fn = Fn_m*params.mm;
end