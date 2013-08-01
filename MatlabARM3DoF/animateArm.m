%Adam Barber
%3/14/2012

%Function animates the 3-dof arm
%Returns the movie

function F = animateArm(times,states,params,dt)
    skip = 50;
    L1 = params.L1; L2 = params.L2; L3 = params.L3;
    
    
    num_points = length(1:skip:length(times));
    
    figure;
    F = moviein(num_points);
   
    for i = 1:skip:length(times)
        %Pull out joint angles
        th1 = states(1,i);
        th2 = states(2,i);
        th3 = states(3,i);
        
        
        %Link 1 starts at 0,0, ends at -L1*s1,-L1*c1
        x1 = L1*sin(th1); y1 = -L1*cos(th1);
        plot([0 x1], [0 y1], '-k.', 'LineWidth',4, 'MarkerSize',20);
        ylim([-L1-L2-L3/2, L1+L2+L3/2]);
        axis equal
        hold on
        
        %Now for Link 2
        x2 = x1 + L2*sin(th1+th2); y2 = y1 - L2*cos(th1+th2);
        plot([x1 x2], [y1 y2], '-k.', 'LineWidth',4, 'MarkerSize',20);
        
        %Link 3
        x31 = x2 + L3/2*sin(th1+th2+th3);
        y31 = y2 - L3/2*cos(th1+th2+th3);
        x32 = x2 - L3/2*sin(th1+th2+th3);
        y32 = y2 + L3/2*cos(th1+th2+th3);
        plot([x31 x32], [y31 y32], '-r.', 'LineWidth',2, 'MarkerSize',10);
        
        %Get frame if you want to animate
        F(i) = getframe();
        pause(dt*skip);
        hold off
    end
    
end