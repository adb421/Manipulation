function [tau1 tau2 tau3] = calcReqTorques(th1,th2,th3,t,params)
    vel1 = diff(th1)/(t(2)-t(1));
    vel2 = diff(th2)/(t(2)-t(1));
    vel3 = diff(th3)/(t(2)-t(1));
    acc1 = diff(vel1)/(t(2)-t(1));
    acc2 = diff(vel2)/(t(2)-t(1));
    acc3 = diff(vel3)/(t(2)-t(1));
    vel1 = [vel1(1), vel1];
    vel2 = [vel2(1), vel2];
    vel3 = [vel3(1), vel3];
    acc1 = [acc1(1), acc1, acc1(end)];
    acc2 = [acc2(1), acc2, acc2(end)];
    acc3 = [acc3(1), acc3, acc3(end)];
    
    Fs = [params.muS1; params.muS2; params.muS3];
    Fv = [params.muD1; params.muD2; params.muD3];
    
    tau1 = zeros(size(t)); tau2 = zeros(size(t)); tau3 = zeros(size(t));
    
    for i = 1:length(t)
        state = [th1(i);th2(i);th3(i);vel1(i);vel2(i);vel3(i)];
        qdot = [vel1(i); vel2(i); vel3(i)];
        qddot = [acc1(i); acc2(i); acc3(i)];
        signQdot = sign(qdot);
        B = inertiaMatrix(state,params);
        C = coriolisMatrix(state,params);
        G = gravityVector(state,params);
        Tau = ...
            B*qddot + C*qdot + G + Fs.*signQdot + Fv.*qdot;
        tau1(i) = Tau(1); tau2(i) = Tau(2); tau3(i) = Tau(3);
    end
end

function B = inertiaMatrix(state, params)
    L1 = params.L1; L2 = params.L2; L3 = params.L3; mm2 = params.mm2;
    mm3 = params.mm3; m1 = params.m1; m2 = params.m2; m3 = params.m3;
    I1 = params.I1; I2 = params.I2; I3 = params.I3;
    J1 = params.J14; J2 = params.J11; J3 = params.J8;
    %th1 = state(1);
    th2 = state(2);
    %th3 = state(3);
    B = zeros(3,3);
    B(1,1) = I1 + I2 + I3 + L2^2*(m2/4 + m3 + mm3) + ...
        L1^2*(m1/4+m2+m3+mm2+mm3) + J1 + J2 + J3 + ...
        L1*L2*(m2+2*(m3 + mm3))*cos(th2);
    B(1,2) = I2 + I3 + L2^2*(m2/4 + m3 + mm3) + J2 + J3 + ...
        0.5*L1*L2*(m2 + 2*m3 + 2*mm3)*cos(th2);
    B(2,1) = B(1,2);
    B(1,3) = J3 + I3;
    B(3,1) = B(1,3);
    B(2,3) = B(1,3);
    B(3,2) = B(1,3);
    B(2,2) = I2 + I3 + L2^2*(m2/4+m3+mm3) + J2 + J3;
    B(3,3) = B(1,3);
end

function C = coriolisMatrix(state, params)
    L1 = params.L1; L2 = params.L2; L3 = params.L3; mm2 = params.mm2;
    mm3 = params.mm3; m1 = params.m1; m2 = params.m2; m3 = params.m3;
    I1 = params.I1; I2 = params.I2; I3 = params.I3;
    J1 = params.J14; J2 = params.J11; J3 = params.J8;
    
    th1 = state(1); th2 = state(2); th3 = state(3);
    th1dot = state(4); th2dot = state(5); th3dot = state(6);
    
    C = zeros(3,3);
    
    C(1,1) = -L1*L2*(m2+2*m3+2*mm3)*sin(th2)*th2dot;
    C(2,1) =  L1*L2/4*(m2+2*m3+2*mm3)*sin(th2)*(2*th1dot - th2dot);
    C(1,2) = -L1*L2/2*(m2+2*m3+2*mm3)*sin(th2)*th2dot;
    C(2,2) =  L1*L2/4*(m2+2*m3+2*mm3)*sin(th2)*th1dot;

end

function G = gravityVector(state, params)
    L1 = params.L1; L2 = params.L2; L3 = params.L3; mm2 = params.mm2;
    mm3 = params.mm3; m1 = params.m1; m2 = params.m2; m3 = params.m3;
    I1 = params.I1; I2 = params.I2; I3 = params.I3;
    J1 = params.J14; J2 = params.J11; J3 = params.J8;
    
    th1 = state(1); th2 = state(2); th3 = state(3);
    th1dot = state(4); th2dot = state(5); th3dot = state(6);
    
    G = zeros(3,1);
    g = 9.81; %m/s^2
    
    
    G(1) = 1/2*g*(L1*(m1+2*(m2+m3+mm2+mm3))*sin(th1) + ...
        L2*(m2+2*(m3+mm3))*sin(th1+th2));
    G(2) = 1/2*g*L2*(m2+2*(m3+mm3))*sin(th1+th2);
end