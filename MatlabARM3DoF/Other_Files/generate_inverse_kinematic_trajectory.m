% Function generates angle data given trajectories in x,y, and theta for
% the end of the 3DoF arm.

% x,y must be meters, theta must be in radians
% t and dt must be in seconds

% Input should be given as 'ELBOW_UP' or 'ELBOW_DOWN'

% flag = 1, good
% flag = -1, arm reach exceeded
% flag = -2, joint angle exceeded


function [theta1 theta2 theta3 flag] =  ...
    generateTrajectory(x, y, theta, armConfig, params)

    flag = 1;
    
    %initialize angles
    theta1 = zeros(size(x));
    theta2 = zeros(size(x));
    theta3 = zeros(size(x));
    
    % Need to do inverse kinematics based on x,y to get theta1 and theta2
    
    %Arm parameters
    L1 = params.L1; % Link length in m
    L2 = params.L2;
    theta1_max = 107.57 * pi/180; %Max angle in radians
    theta2_max = 139.64 * pi/180; %Max angle in radians
    
    %Initial test for reachable workspace
    %Check furthest distance
    arm_reach = sqrt(x.^2 + y.^2);
    if (max(arm_reach) > L1 + L2)
        disp('Arm reach exceeded');
        flag = -1;
        return;
    end
    
    % From Wei's writeup:
    % c2 = cos(theta2), s2 = sin(theta2)
    % c2 = (x^2 + y^2 - L1^2 - L2^2)/2*L1*L21
    % s2 = +/-sqrt(1-c2^2), elbow down => positive
    % theta2 = arctan(s2/c2)
    % r = sqrt(x^2 + y^2), d = L1*c2 + L2
    % theta1 = arctan(y/x) + arctan(+/-sqrt(r^2-d^2)/d) - theta2 elbow down
    % => positive
    % From my work:
    % This is for x positive to the left, y positive down, theta = 0
    % pointing in the positive y direction, z is out of the plane (so
    % positive theta is counter-clockwise). This is how I have defined the
    % reference frame for the robot.
    % x = -L1*sin(theta1) - L2*sin(theta1+theta2)
    % y = L1*cos(theta1) + L2*cos(theta1+theta2)
    % c2 = cos(theta1), s2 = sin(theta2)
    % c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2)
    % s2 = +/- sqrt(1-c2^2), elbow down -> positive 
    % theta2 = atan2(s2,c2)
    % k1 = L1 + L2*c2; k2 = L2*s2;
    % can show: theta1 = atan2(-x,y) - atan2(k2,k1)
    c2 = (x.^2 + y.^2 - L1^2 - L2^2)/(2*L1*L2);
    if(strcmp(armConfig, 'ELBOW_DOWN'))
        s2 = -sqrt(1-c2.^2);
    else
        s2 = sqrt(1-c2.^2);
    end
    theta2 = atan2(s2,c2);
    alpha = atan2(x,y);
    k = s2*L2./arm_reach;
    if strcmp(armConfig, 'ELBOW_DOWN')
        theta1 = pi + (-atan2(-sqrt(1-k.^2),k) - alpha);
    else
        theta1 = pi + (atan2(sqrt(1-k.^2), k) - alpha);
    end
    while(~(max(theta1) <= 2*pi && ...
            min(theta1) >= -2*pi))
        if(max(theta1) >= 2*pi)
            theta1 = theta1 - 2*pi;
        else
            theta1 = theta1 + 2*pi;
        end
    end
    while(~(max(theta2) <= 2*pi && ...
            min(theta2) >= -2*pi))
        if(max(theta2)>= 2*pi)
            theta2 = theta2 - 2*pi;
        else
            theta2 = theta2 + 2*pi;
        end
    end
    theta1 = unwindTraj(theta1);
    theta2 = unwindTraj(theta2);
    for i = 1:length(x)
        %Test angle limits
        if abs(theta1(i)) > theta1_max
            flag = -2;
%             disp('Joint 1 limit exceeded');
        %    return;
        elseif abs(theta2(i)) > theta2_max
            flag = -2;
%             disp('Joint 2 limit exceeded');
        %    return;
        end
    end
    
    %Note, theta = theta1 + theta2 + theta3
    theta3 = theta - theta1 - theta2;
end

%Makes sure you don't have discontinuities from keeping things from zero to
%2pi (or whatever range it keeps it)
function traj = unwindTraj(traj)
    changed = 1;
    if(length(traj) == 1)
        changed = 0;
    end
    while(changed)
        changed = 0;
        for i = 1:(length(traj)-1)
            if (traj(i) - traj(i+1) >= 6)
                changed = 1;
                traj(i+1:end) = traj(i+1:end) + 2*pi;
                break
            elseif (traj(i+1) - traj(i) >= 6)
                changed = 1;
                traj(i+1:end) = traj(i+1:end) - 2*pi;
                break
            end
        end
    end
end