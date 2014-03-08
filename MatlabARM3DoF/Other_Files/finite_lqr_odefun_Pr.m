%Calcs P and r
function Prdot = finite_lqr_odefun_Pr(t,Pr,xd,ud,td,Q,R,params)
    xDes = zeros(8,1); uDes = zeros(3,1);
    for i = 1:8
        xDes(i) = interp1(td,xd(i,:),t);
    end
    for i = 1:3
        uDes(i) = interp1(td,ud(i,:),t);
    end
    [A,B] = onePointRollingLinear(xDes,uDes,params);
    P = reshape(Pr(1:64),8,8);
    r = Pr(65:end);
    Pdot = -(A'*P + P*A - P*B*inv(R)*B'*P + Q);
    rdot = -(A' - P*B*inv(R)*B')*r;% + Q*xd;
    Pdot = reshape(Pdot,64,1);
    Prdot = [Pdot;rdot];
end