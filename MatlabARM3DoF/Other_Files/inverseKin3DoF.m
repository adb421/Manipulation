% Function gives the inverse kinematics of the arm given the endpoint configuration
% and the configuration of the arm as a string either 'ELBOW_DOWN' or 'ELBOW_UP'
% and the params structure
function [theta1, theta2, theta3] = inverseKin3DoF(x,y,theta, armConfig, params)
    L1 = params.L1;
    L2 = params.L2;
    if nargin == 3
        armConfig = 'ELBOW_UP';
    end
    if nargin == 2
        theta = NaN;
    end
    
    c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
    if strcmp(armConfig, 'ELBOW_DOWN')
        s2 = sqrt(1 - c2^2);
    else
        s2 = -sqrt(1 - c2^2);
    end
    if(~isreal(s2) && ~isreal(c2))
        theta2 = atan2(s2,c2);
    else
        disp(['link lengths exceeded? r = ' num2str(x^2+y^2) ...
            ' and rmax = ' num2str(L1^2 + L2^2)]);
        return;
    end
    r = sqrt(x^2 + y^2);
%     d = L1*c2 + L2;
%     if strcmp(armConfig, 'ELBOW_DOWN')
%         theta1 = atan2(y,x) + atan2(sqrt(r^2-d^2),d) ...
%             - theta2 + pi/2;
%     else
%         theta1 = atan2(y,x) + atan2(-sqrt(r^2-d^2),d) ...
%             - theta2 + pi/2;
%     end
    alpha = atan2(y,x);
    k = s2*L2/r;
    if strcmp(armConfig, 'ELBOW_DOWN')
        theta1 = atan2(sqrt(1-k^2), k) + alpha;
    else
        theta1 = -atan2(-sqrt(1-k^2),k) + alpha;
    end
    
    theta3 = theta - theta2 - theta1;
end