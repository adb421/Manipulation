function cost = ...
    optimMotorGainsCost(x, xDes, yDes, thDes, t, armConfig, params)

kp = [x(1) x(2) 0];
kd = [x(3) x(4) 0];
ki = [x(5) x(6) 0];

[pos1 pos2 pos3 th1 th2 th3] = ...
    sendTrajectory(xDes, yDes, thDes, t, armConfig, kp, kd, ki, params);

%calculate cost

cost = sum((pos1 - th1).^2) + sum((pos2 - th2).^2) + sum((pos3 - th3).^2);