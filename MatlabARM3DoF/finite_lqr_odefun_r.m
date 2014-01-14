%Calculates the derivative of r for the Ricatti equation
function rdot = finite_lqr_odefun_r(t,r,xd,ud,Q,R,P,params)
    [A,B] = onePointRollingLinear(xd,ud,params);
%     rdot = -(transpose(A - B*inv(R)*B'*P)*r + Q*xd);
    rdot = -(A' - P*B*inv(R)*B')*r + Q*xd;
%     Q*xd + (P*B*inv(R)*B' - A')*r;
end