%Calculates the derivative of P for the Ricatti equation
function Pdot = finite_lqr_odefun_P(t,P,A,B,Q,R,params)
    P = reshape(P,8,8);
    Pdot = -(A'*P + P*A - P*B*inv(R)*B'*P + Q);
    Pdot = reshape(Pdot,64,1);
end