%Calculates the derivative of P for the Ricatti equation
function Pdot = finite_lqr_odefun(t,P,xd,ud,Q,R)
    [A,B] = onePointRollingLinear(xd,ud);
    P = reshape(P,8,8);
    Pdot = -(A'*P + P*A - P*B*inv(R)*B'*P + Q);
end