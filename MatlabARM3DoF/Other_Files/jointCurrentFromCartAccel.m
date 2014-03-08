%Given the state and control (accelerations of the manipulator) as functions
%of time and the parameters structure, calculates the accelerations of teh
%joints as well as the required currents at the motors
function [currents, accels] = jointCurrentFromCartAccel(u,state,params)
    xm = state(1,:);
    xmd = state(2,:);
    ym = state(3,:);
    ymd = state(4,:);
    thm = state(5,:);
    thmd = state(6,:);
    xmdd = u(1,:);
    ymdd = u(2,:);
    thmdd = u(3,:);
    currents = zeros(3,length(xm));
    accels = zeros(3,length(xm));
    for i = 1:length(xm)
        %Get joint positions
        [th1, th2, th3] = inverseKin3DoF(xm(i),ym(i),thm(i),'ELBOW_UP',params);
        %Get joint velocities
        Jinv = jacobianInv(th1,th2,params);
        thd = Jinv*[xmd(i);ymd(i);thmd(i)];
        th1d = thd(1); th2d = thd(2); th3d = thd(3);
        %Calculate Mass Matrix, coriolois, Gravity, Friction
        M = massMatrix(th2,params);
        C = coriolisMatrix(th2,th2d,th1d,params);
        G = gravityMatrix(th1, th2, params);
        Ff = frictionMatrix(th1d,th2d,th3d,params);
        Jdot = jacobianDot(th1,th1d,th2,th2d,params);
        
        thdd = Jinv*([xmdd(i); ymdd(i); thmdd(i)] ...
            - Jdot*Jinv*[xmd(i); ymd(i); thmd(i)]);
        %Arm equations of motion
        torques = M*thdd + C + G + Ff;
        %Calculate joint currents and store them
        currents(1,i) = torques(1)/params.km1;
        currents(2,i) = torques(2)/params.km2;
        currents(3,i) = torques(3)/params.km3;
        accels(:,i) = thdd;
    end
end

function M = massMatrix(th2,params)
    M = zeros(3);
    M(1,1) = params.I1 + params.I2 + params.I3 + params.L2^2*(params.m2/4 + ...
        params.m3 + params.mm3) + params.L1^2*(params.m1/4 + params.m2 + ...
        params.m3 + params.mm2 + params.mm3) + params.J11 + params.J14 + ...
        params.J8 + params.L1*params.L2*(params.m2 + 2*(params.m3 + ...
        params.mm3))*cos(th2);
    M(1,2) = params.I2 + params.I3 + params.L2^2*(params.m2/4 + params.m3 + ...
        params.mm3) + params.J11 + params.J8 + ...
        (params.L1*params.L2*(params.m2 + 2*(params.m3 + params.mm3))*cos(th2))/2;
    M(1,3) = params.I3 + params.J8;
    M(2,2) = params.I2 + params.I3 + params.J11 + params.J8 + ...
        params.L2^2*(params.m2/4 + params.m3 + params.mm3);
    M(2,3) = M(1,3);
    M(3,3) = M(1,3);
    M(2,1) = M(1,2);
    M(3,2) = M(2,3);
    M(3,1) = M(1,3);
end

function C = coriolisMatrix(th2,th2d,th1d,params)
    C = zeros(3,1);
    C(1) = -(params.L1*params.L2*(params.m2 + 2*(params.m3 + params.mm3))*...
        sin(th2)*th2d*(2*th1d+th2d))/2;
    C(2) = (params.L1*params.L2*(params.m2 + 2*(params.m3 + params.mm3))*...
        sin(th2)*th1d^2)/2;
end

function G = gravityMatrix(th1, th2, params)
    G = zeros(3,1);
    G(1) = -(params.g*(params.L1*(params.m1 + 2*(params.m2 + params.m3 + ...
        params.mm2 + params.mm3))*cos(th1) + params.L2*(params.m2 + ...
        2*(params.m3 + params.mm3))*cos(th1+th2)))/2;
    G(2) = -(params.g*params.L2*(params.m2 + 2*(params.m3 + params.mm3))*cos(th1+th2))/2;
    G(3) = 0;
end

function Jdot = jacobianDot(th1,th1d,th2,th2d,params)
    Jdot = zeros(3,3);
    Jdot(1,1) = params.L1*cos(th1)*th1d + params.L2*cos(th1+th2)*(th1d+th2d);
    Jdot(2,1) = params.L1*sin(th1)*th1d + params.L2*sin(th1+th2)*(th1d+th2d);
    Jdot(2,1) = params.L2*cos(th1+th2)*(th1d+th2d);
    Jdot(2,2) = params.L2*sin(th1+th2)*(th1d+th2d);
end

function Jinv = jacobianInv(th1,th2,params)
    Jinv = zeros(3,3);
    Jinv(1,1) = -cos(th1+th2)*csc(th2)/params.L1;
    Jinv(2,1) = (cos(th1)/params.L2 + cos(th1+th2)/params.L1)*csc(th2);
    Jinv(3,1) = -cos(th1)*csc(th2)/params.L2;
    Jinv(2,1) = -csc(th2)*sin(th1+th2)/params.L1;
    Jinv(2,2) = csc(th2)*(sin(th1)/params.L2 + sin(th1+th2)/params.L1);
    Jinv(2,3) = -csc(th2)*sin(th1)/params.L2;
    Jinv(3,3) = 1;
end

function Ff = frictionMatrix(th1d,th2d,th3d,params)
    Ff = zeros(3,1);
    Ff(1) = params.muD1*th1d + params.muS1*sign(th1d);
    Ff(2) = params.muD2*th2d + params.muS2*sign(th2d);
    Ff(3) = params.muD3*th3d + params.muS3*sign(th3d);
end