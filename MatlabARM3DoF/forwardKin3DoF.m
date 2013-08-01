function [x, y, theta] = forwardKin3DoF(theta1, theta2, theta3, params)

%     x = zeros(size(theta1));
%     y = zeros(size(theta2));
    
    %Arm Parameters
    L1 = params.L1;
    L2 = params.L2;
    
%     x = L1*sin(theta1) + L2*sin(theta1+theta2);
%     y = -L1*cos(theta1) - L2*cos(theta1+theta2);
    x = -L1*cos(theta1) - L2*cos(theta1+theta2);
    y = -L1*sin(theta1) - L2*sin(theta1+theta2);
    theta = theta1 + theta2 + theta3;
end