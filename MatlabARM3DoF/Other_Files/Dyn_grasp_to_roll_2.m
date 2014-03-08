%Simulation of a dynamic grasp to a one point roll

%t is the time vector, x is the state vector (12 dimensional), and M is the
%matlab movie
%xdGrasp is the desired dynamic grasp trajectory (6xn) tdGrasp is the time
%vector associated with that trajectory (1xn) xdRoll is the desired one point
%rolling trajectory (8xm) and tdRoll is the time vector associated with that
%trajectory (1xm), params is a parameters structure from
%ParametersFunction() most likely, don't forget to set params.s1 manually
%(the left contact point)
function [t, x, M] = Dyn_grasp_to_roll_2(xdGrasp,tdGrasp,udGrasp,xdRoll, tdRoll, ...
                                         udRoll, params)
                                     
%Assume xd comes as a 6 dim vector of object states
%Pull out geometric terms to get manipulator states
    lo = params.lo; wo = params.wo; lm = params.lm;
    wm = params.wm; s1 = params.s1;% lc = params.lc;
    
    xdRoll = one_point_roll_min_mu(xdRoll,tdRoll,params);
    
    th0 = xdGrasp(5,1); %Pull out initial object angle, it makes things easy
                   %Set up initial state vector for integration, need all 12 state variables
    x0 = [xdGrasp(1,1) - lo*cos(th0) + wo*sin(th0) - s1*cos(th0) + lm*cos(th0) + ...
          wm*sin(th0); %xm
          ((wm + wo)*cos(th0) + (-lm + lo + s1)*sin(th0))*xdGrasp(6,1) + xdGrasp(2,1); ...
    %xmd
          xdGrasp(3,1) - wo*cos(th0) - lo*sin(th0) - s1*sin(th0) - wm*cos(th0) ...
          + lm*sin(th0); %ym
          ((lm - lo - s1)*cos(th0) + (wm + wo)*sin(th0))*xdGrasp(6,1) + xdGrasp(4,1); ...
          xdGrasp(5,1); %thm
          xdGrasp(6,1); %thmd
          xdGrasp(:,1)]; %Include the object states
    
    %Now, x is the full 12-dim state vector.
    %Set up the grasp and roll times
    %The trajectory given is just for the grasp, so we also need to add a
    %time vector for the roll
    td = [tdGrasp tdRoll]; %Full time vector is both.
    
    %Dynamic grasp time!
    %No more loops for this nonsense. Let ode45 do its thing
    [tGrasp, xGrasp] = ode45(@(t,y) dynamicGraspODE(t,y,params,xdGrasp, ...
                                                    udGrasp,tdGrasp),tdGrasp,x0);
    tGrasp = tGrasp'; xGrasp = xGrasp';

    %Cost function for one point rolling is quadratic
    Q = eye(8); R = eye(3);
    Q(1,1) = 50;
    Q(3,3) = 1000;
    Q(5,5) = 100;

    %Calculate Kbal as a function of time
    %Need to integrate backwards
    xdFlip = fliplr(xdRoll);
    udFlip = fliplr(udRoll);
    tdFlip = fliplr(tdRoll);
    [~, PrFlip] = ode45(@(t,y) lqr_odefun_Pr(t,y,xdFlip,udFlip,tdFlip,Q,R,params), ...
                          tdFlip, zeros(72,1));
    PrRoll = fliplr(PrFlip');
    
    %Integrate one-point-roll
    %Start from the end of the grasp
    [tRoll, xRoll] = ode45(@(t,y) ...
        onePointRolling(t,y,xdRoll, udRoll, PrRoll, tdRoll, R, params), ...
        tdRoll,xGrasp(:,end));
    tRoll = tRoll';
    xRoll = xRoll';
    
    t = [tGrasp tRoll];
    x = [xGrasp xRoll];
    %Calculate contact forces
    Ff = zeros(1,length(tGrasp) + length(tRoll));
    Fn = zeros(1,length(tGrasp) + length(tRoll));
    s = zeros(1,length(tGrasp) + length(tRoll));
    for i = 1:length(tGrasp)
        [~, FfT, FnT, sT] = dynamicGraspODE(t(i), x(:,i), params, xdGrasp, ...
                                        udGrasp, tdGrasp);
        Ff(i) = FfT;
        Fn(i) = FnT;
        s(i) = sT;
    end
    for i = length(tGrasp) + 1: length(Ff)
        [~,~,FfT,FnT] = ...
            onePointRolling(t(i),x(:,i),xdRoll, udRoll, PrRoll, tdRoll, R, params);
        Ff(i) = FfT;
        Fn(i) = FnT;
        s(i) = s1;
    end
    
    %Set up desired trajectories
    lc = params.lc; thc = params.objAngle;
    xd = zeros(12,length(t));
    xd(:,1:length(tdGrasp)) = [xdGrasp(1,:) - lo*cos(xdGrasp(5,:)) + wo*sin(xdGrasp(5,:)) ...
        - s1*cos(xdGrasp(5,:)) + lm*cos(xdGrasp(5,:)) +  ...
        wm*sin(xdGrasp(5,:)); %xm
        ((wm + wo)*cos(xdGrasp(5,:)) + (-lm + lo + ...
        s1)*sin(xdGrasp(5,: ...
        ))).*xdGrasp(6,:) + xdGrasp(2,:); %xmd
        xdGrasp(3,:) - wo*cos(xdGrasp(5,:)) - lo*sin(xdGrasp(5,:)) ...
        - s1*sin(xdGrasp(5,:)) - wm*cos(xdGrasp(5,:)) ...
        + lm*sin(xdGrasp(5,:)); %ym
        ((lm - lo - s1)*cos(xdGrasp(5,:)) + (wm + wo)*sin(xdGrasp(5,:))).*xdGrasp(6,:) + xdGrasp(4,:); ...
        xdGrasp(5,:); %thm
        xdGrasp(6,:); %thmd
        xdGrasp]; %Include the object states
    xd(:,length(tdGrasp)+1:end) = ...
        [(lm - s1)*cos(xdRoll(1,:)) - lc*cos(thc ...
        + xdRoll(7,:)) + ...
        wm*sin(xdRoll(1,:)) + xdRoll(3,:); ... %xm
        (wm*cos(xdRoll(1,:)) + (s1 - lm)*sin(xdRoll(1,:))).*xdRoll(2,:) + ...
        lc*sin(thc+xdRoll(7,:)).*xdRoll(8,:) + xdRoll(4,:); ... %xmd
        -wm*cos(xdRoll(1,:)) + (lm - s1)*sin(xdRoll(1,:)) - lc*sin(thc + ...
        xdRoll(7,:)) + ...
        xdRoll(5,:); ... %ym
        ((lm - s1)*cos(xdRoll(1,:)) + wm*sin(xdRoll(1,:))).*xdRoll(2,:) - ...
        lc*cos(thc + xdRoll(7,:)).*xdRoll(8,:) + xdRoll(5,:); ... %ymd
        xdRoll];
    
    ud = [udGrasp udRoll];
    %Have our desired trajectory, our desired control, our time vector,
    %our integrated state, and our forces. Time to plot and animate.
    
    figure;
    subplot(3,1,1);
    title('X position of object CoM')
    plot(t,xd(7,:),t,x(7,:));
    legend('Desired','Actual');
    xlabel('Time (s)'); ylabel('Position (m)');
    subplot(3,1,2);
    title('Y position of object CoM')
    plot(t,xd(9,:),t,x(9,:));
    legend('Desired','Actual');
    xlabel('Time (s)'); ylabel('Position (m)');
    subplot(3,1,3);
    title('Angular position of object CoM')
    plot(t,xd(11,:),t,x(11,:));
    legend('Desired','Actual');
    xlabel('Time (s)'); ylabel('Position (rad)');
    mu = params.mu;
    figure;
    title('Contact forces')
    plot(t,Ff,'-r',t,mu*Fn,'-b',t,abs(Ff),'--r');
    xlabel('Time (s)'); ylabel('Force (N)');
    legend('Friction force','Normal force*$\mu$','abs(Friction force)');    
    
    figure;
    set(gca,'fontname','Bitstream Charter','fontsize',18);
    hold on
    xlim([-0.2 0.2])
    ylim([-0.2 0.2])
    axis square
%     axis off
    %bottom left, bottom right, top right, top left
    xdR = xdRoll(3,end); thdR = xdRoll(7,end); ydR = xdRoll(5,end);
    xverts = [xdR - params.lo*cos(thdR) + params.wo*sin(thdR); ...
              xdR + params.lo*cos(thdR) + params.wo*sin(thdR); ...
              xdR + params.lo*cos(thdR) - params.wo*sin(thdR); ...
              xdR - params.lo*cos(thdR) - params.wo*sin(thdR)];
    yverts = [ydR - params.wo*cos(thdR) - params.lo*sin(thdR); ...
              ydR - params.wo*cos(thdR) + params.lo*sin(thdR); ...
              ydR + params.wo*cos(thdR) + params.lo*sin(thdR);
              ydR + params.wo*cos(thdR) - params.lo*sin(thdR)];
    rollGoal = patch(xverts,yverts,'g');
    set(rollGoal,'EdgeColor','g','LineWidth',2);
    set(rollGoal,'FaceColor',[1 1 1]);
    xdG = xdGrasp(1,end); thdG = xdGrasp(5,end); ydG = xdGrasp(3,end);
    xverts = [xdG - params.lo*cos(thdG) + params.wo*sin(thdG); ...
              xdG + params.lo*cos(thdG) + params.wo*sin(thdG); ...
              xdG + params.lo*cos(thdG) - params.wo*sin(thdG); ...
              xdG - params.lo*cos(thdG) - params.wo*sin(thdG)];
    yverts = [ydG - params.wo*cos(thdG) - params.lo*sin(thdG); ...
              ydG - params.wo*cos(thdG) + params.lo*sin(thdG); ...
              ydG + params.wo*cos(thdG) + params.lo*sin(thdG);
              ydG + params.wo*cos(thdG) - params.lo*sin(thdG)];
    graspGoal = patch(xverts,yverts,'g');
    set(graspGoal,'EdgeColor','g','LineWidth',2,'LineStyle','--');
    set(graspGoal,'FaceColor',[1 1 1]);
    xm = x(1,:); ym = x(3,:); thm = x(5,:);
    xo = x(7,:); yo = x(9,:); tho = x(11,:);
    xverts = [xm(1) + params.lm*cos(thm(1)) - params.wm*sin(thm(1)); ...
              xm(1) - params.lm*cos(thm(1)) - params.wm*sin(thm(1))];
%     xverts = [xm(1) - params.lm*cos(thm(1)) + params.wm*sin(thm(1)); ...
%               xm(1) + params.lm*cos(thm(1)) + params.wm*sin(thm(1)); ...
%               xm(1) + params.lm*cos(thm(1)) - params.wm*sin(thm(1)); ...
%               xm(1) - params.lm*cos(thm(1)) - params.wm*sin(thm(1))];
%     yverts = [ym(1) - params.wm*cos(thm(1)) - params.lm*sin(thm(1)); ...
%               ym(1) - params.wm*cos(thm(1)) + params.lm*sin(thm(1)); ...
%               ym(1) + params.wm*cos(thm(1)) + params.lm*sin(thm(1)); ...
%               ym(1) + params.wm*cos(thm(1)) - params.lm*sin(thm(1))];
    yverts = [ym(1) + params.wm*cos(thm(1)) + params.lm*sin(thm(1)); ...
              ym(1) + params.wm*cos(thm(1)) - params.lm*sin(thm(1))];
    contactForceBase = [xverts(2) + s(1)*cos(thm(1)); yverts(2) + s(1)* ...
        sin(thm(1))];
    manip = line(xverts,yverts);
    set(manip,'Color','r','LineWidth',3);
    xverts = [xo(1) - params.lo*cos(tho(1)) + params.wo*sin(tho(1)); ...
              xo(1) + params.lo*cos(tho(1)) + params.wo*sin(tho(1)); ...
              xo(1) + params.lo*cos(tho(1)) - params.wo*sin(tho(1));
              xo(1) - params.lo*cos(tho(1)) - params.wo*sin(tho(1))];
    yverts = [yo(1) - params.wo*cos(tho(1)) - params.lo*sin(tho(1)); ...
              yo(1) - params.wo*cos(tho(1)) + params.lo*sin(tho(1)); ...
              yo(1) + params.wo*cos(tho(1)) + params.lo*sin(tho(1));
              yo(1) + params.wo*cos(tho(1)) - params.lo*sin(tho(1))];
    obj = patch(xverts,yverts,'b');
    set(obj,'EdgeColor','b','LineWidth',3);
    set(obj,'FaceColor',[1 1 1]);
    coneLength = 0.05;
    %Do friction cones also
    mu = params.mu;
    beta = atan(mu);
    base1 = [xverts(1); yverts(1)];
    left1 = base1 + coneLength*[cos(thm(1) + pi/2 + beta); sin(thm(1) + pi/2 + ...
                                                      beta)];
    right1 = base1 + coneLength*[cos(thm(1) + pi/2 - beta); sin(thm(1) + pi/2 ...
                                                      - beta)];
    base2 = [xverts(2); yverts(2)];
    left2 = base2 + coneLength*[cos(thm(1) + pi/2 + beta); sin(thm(1) + pi/2 + ...
                                                      beta)];
    right2 = base2 + coneLength*[cos(thm(1) + pi/2 - beta); sin(thm(1) + pi/2 ...
                                                      - beta)];
    forceMag = coneLength; %norm([Ff(1) Fn(1)]);
    forceAngle = thm(1) + pi/2  - atan2(Ff(1),Fn(1));
    contactForceEnd = contactForceBase + forceMag*[cos(forceAngle); ...
        sin(forceAngle)];
    leftLine1 = line([base1(1) left1(1)], [base1(2), left1(2)],'Color','k', ...
        'LineWidth',2);
    rightLine1 = line([base1(1) right1(1)],[base1(2), right1(2)],'Color','k', ...
        'LineWidth',2);
    leftLine2 = line([base2(1) left2(1)], [base2(2), left2(2)],'Color','k', ...
        'LineWidth',2);
    rightLine2 = line([base2(1) right2(1)],[base2(2), right2(2)],'Color','k', ...
        'LineWidth',2);
    contactForceLine = line([contactForceBase(1), contactForceEnd(1)], ...
        [contactForceBase(2), contactForceEnd(2)],'Color','r', ...
        'LineWidth',2);
    M(length(1:length(t))) = struct('cdata',[],'colormap',[]);
    j = 1;
    for i=1:1:length(t)
%         if(mod((t(i) - 0.01)*100,30) == 0 || t(i) == tdGrasp(end) || t(i) == tdRoll(2))
%             disp(t(i))
%         end
       if(i == 92 || i == 242)
           disp(t(i))
       end
%         xverts = [xm(i) - params.lm*cos(thm(i)) + params.wm*sin(thm(i)); ...
%                   xm(i) + params.lm*cos(thm(i)) + params.wm*sin(thm(i)); ...
%                   xm(i) + params.lm*cos(thm(i)) - params.wm*sin(thm(i)); ...
%                   xm(i) - params.lm*cos(thm(i)) - params.wm*sin(thm(i))];
%         yverts = [ym(i) - params.wm*cos(thm(i)) - params.lm*sin(thm(i)); ...
%                   ym(i) - params.wm*cos(thm(i)) + params.lm*sin(thm(i)); ...
%                   ym(i) + params.wm*cos(thm(i)) + params.lm*sin(thm(i)); ...
%                   ym(i) + params.wm*cos(thm(i)) - params.lm*sin(thm(i))];
        xverts = [xm(i) + params.lm*cos(thm(i)) - params.wm*sin(thm(i)); ...
            xm(i) - params.lm*cos(thm(i)) - params.wm*sin(thm(i))];
        yverts = [ym(i) + params.wm*cos(thm(i)) + params.lm*sin(thm(i)); ...
            ym(i) + params.wm*cos(thm(i)) - params.lm*sin(thm(i))];
        contactForceBase = [xverts(2) + s(i)*cos(thm(i)); yverts(2) + s(i)* ...
            sin(thm(i))];
        set(manip,'XData',xverts,'YData',yverts);
%         manip = patch(xverts,yverts,'r');
%         set(manip,'EdgeColor','r','LineWidth',3);
%         set(manip,'FaceColor',[1 1 1]);
        xverts = [xo(i) - params.lo*cos(tho(i)) + params.wo*sin(tho(i)); ...
                  xo(i) + params.lo*cos(tho(i)) + params.wo*sin(tho(i)); ...
                  xo(i) + params.lo*cos(tho(i)) - params.wo*sin(tho(i));
                  xo(i) - params.lo*cos(tho(i)) - params.wo*sin(tho(i))];
        yverts = [yo(i) - params.wo*cos(tho(i)) - params.lo*sin(tho(i)); ...
                  yo(i) - params.wo*cos(tho(i)) + params.lo*sin(tho(i)); ...
                  yo(i) + params.wo*cos(tho(i)) + params.lo*sin(tho(i));
                  yo(i) + params.wo*cos(tho(i)) - params.lo*sin(tho(i))];
        set(obj,'XData',xverts,'YData',yverts);
        % obj = patch(xverts,yverts,'b');
        %     set(obj,'EdgeColor','b','LineWidth',3);
        %     set(obj,'FaceColor',[1 1 1]);
%         %Do friction cones also
        mu = params.mu;
        beta = atan(mu);
        base1 = [xverts(1); yverts(1)];
        left1 = base1 + coneLength*[cos(thm(i) + pi/2 + beta); sin(thm(i) + pi/2 + ...
            beta)];
        right1 = base1 + coneLength*[cos(thm(i) + pi/2 - beta); sin(thm(i) + pi/2 ...
            - beta)];
        set(leftLine1,'XData',[base1(1) left1(1)], ...
            'YData', [base1(2), left1(2)]);
        set(rightLine1,'XData',[base1(1) right1(1)], ...
            'YData', [base1(2), right1(2)]);
        if(i <= length(tGrasp))
            base2 = [xverts(2); yverts(2)];
            left2 = base2 + coneLength*[cos(thm(i) + pi/2 + beta); sin(thm(i) + pi/2 + ...
                beta)];
            right2 = base2 + coneLength*[cos(thm(i) + pi/2 - beta); sin(thm(i) + pi/2 ...
                - beta)];
            set(leftLine2,'XData',[base2(1) left2(1)], ...
                'YData', [base2(2), left2(2)]);
            set(rightLine2,'XData',[base2(1) right2(1)], ...
                'YData', [base2(2), right2(2)]);
        else
            set(leftLine2,'XData',[base1(1) base1(1)],'YData', ...
                [base1(2) base1(2)]);
            set(rightLine2,'XData',[base1(1) base1(1)],'YData', ...
                [base1(2) base1(2)]);
        end
        forceMag = coneLength; %norm([Ff(i) Fn(i)]);
        forceAngle = thm(i) + pi/2 - atan2(Ff(i),Fn(i));
        contactForceEnd = contactForceBase + forceMag*[cos(forceAngle); ...
            sin(forceAngle)];
        set(contactForceLine,'XData', ...
            [contactForceBase(1), contactForceEnd(1)], ...
            'YData', [contactForceBase(2), contactForceEnd(2)]);
        M(j) = getframe;
        j = j+1;
    end
    movie(M,2,100)    
end


function [dx, Ff, Fn, s] = dynamicGraspODE(t, x, params, xd, ud, td)
    kp1 = 150; kp2 = 250; kp3 = 150;
    kd1 = 30; kd2 = 30; kd3 = 5;
    %Set up feedback matrix
    K = [kp1, kd1, 0, 0, 0, 0; ...
         0, 0, kp2, kd2, 0, 0; ...
         0, 0, 0, 0, kp3, kd3];    
    %Pull out relevant parameters
    mu = params.mu;
    mm = params.mm; Jm = params.Jm; wm = params.wm; lm = params.lm;
    wo = params.wo; lo = params.lo; s1 = params.s1; g = params.g;
    lc = params.lc; thc = params.objAngle; mo = params.mo;
    Jo = params.Jo;
    xDes = [interp1(td,xd(1,:),t); ...
            interp1(td,xd(2,:),t); ...
            interp1(td,xd(3,:),t); ...
            interp1(td,xd(4,:),t); ...
            interp1(td,xd(5,:),t); ...
            interp1(td,xd(6,:),t)];
    uDes = [interp1(td,ud(1,:),t); ...
            interp1(td,ud(2,:),t); ...
            interp1(td,ud(3,:),t)];
    %Calculate control:
    error = xDes - x(7:12);
    u = K*error + uDes;
    %This control is desired object accelerations, calculate
    %desired manipulator accelerations
    thmdd = u(3);
    xmdd = -((lm - s1)*cos(x(5)) + wm*sin(x(5)))*x(6)^2 + lc*cos(thc+x(11))*x(12)^2 ...
           + (wm*cos(x(5)) + (s1-lm)*sin(x(5)))*thmdd + lc*sin(thc + x(11))*u(3) ...
           + u(1);
    ymdd = (wm*cos(x(5)) + (s1-lm)*sin(x(5)))*x(6)^2 + lc*sin(thc+x(11))*x(12)^2 ...
           + ((lm-s1)*cos(x(5)) + wm*sin(x(5)))*thmdd - lc*cos(thc+x(11))*u(3) ...
           + u(2);
    Ff = mo*(u(1)*cos(x(11)) + (g + u(2))*sin(x(11)));
    Fn = mo*((g + u(2))*cos(x(11)) - u(1)*sin(x(11)));
    s = lo + (Jo*u(3) - Ff*wo)/Fn;
    s = s + s1;
    
    dx = zeros(12,1);
    %Dynamics are easy
    dx(1) = x(2); dx(3) = x(4); dx(5) = x(6); dx(7) = x(8); dx(9) = x(10);
    dx(11) = x(12);
    dx(2) = xmdd; dx(4) = ymdd; dx(6) = thmdd;
    dx(8) = u(1); dx(10) = u(2); dx(12) = u(3);
end

function PrDot = lqr_odefun_Pr(t,Pr,xd,ud,td,Q,R,params)
    uDes = [interp1(td,ud(1,:),t); interp1(td,ud(2,:),t); interp1(td,ud(3,:),t)];
    xDes = [interp1(td,xd(1,:),t); interp1(td,xd(2,:),t); interp1(td,xd(3,:),t); ...
            interp1(td,xd(4,:),t); interp1(td,xd(5,:),t); interp1(td,xd(6,:),t); ...
            interp1(td,xd(7,:),t); interp1(td,xd(8,:),t)];
    [A, B] = onePointRollingLinear(xDes,uDes,params);
    P = reshape(Pr(1:64),8,8);
    r = Pr(65:end);
    Pdot = -(A'*P + P*A - P*B*inv(R)*B'*P + Q);
    rdot = -(A' - P*B*inv(R)*B')*r;
    Pdot = reshape(Pdot,64,1);
    PrDot = [Pdot; rdot];
end
