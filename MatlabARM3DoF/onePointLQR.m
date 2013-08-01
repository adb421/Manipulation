function K = onePointLQR()
lo = 0.0425; wo = 0.0255; lm = 0.09; wm = 0.0255;
g = 9.81*sin(0.4);
lc = sqrt(lo^2 + wo^2);
s1 = lm;

T = 5;

R1 = 10; R2 = 10; R3 = 10;
Q1 = 50; Q2 = 10;
Q3 = 10;  Q4 = 10; 
Q5 = 75;  Q6 = 10; 
Q7 = 50; Q8 = 10;

A = zeros(8,8);
A(1,2) = 1; A(3,4) = 1; A(5,6) = 1; A(7,8) = 1;
A(4,7) = -3*g/4; A(8,7) = 3*g/(4*lc);

B = zeros(8,3);
B(2,3) = 1; B(4,1) = .25;
B(4,3) = -wm/4; B(6,2) = 1;
B(6,3) = s1 - lm;B(8,1) = 3/(4*lc);
B(8,3) = -3*wm/(4*lc);

R = diag([R1 R2 R3]);
Q = diag([Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8]);

[K,~,e] = lqr(A,B,Q,R);
min(e)
max(e)
end