clear
% Animates solutions found by AStar in the forms of "Solution.txt"
saveMovie = 0;
backwards_int = 0;
AStar = 0;
RGRRT = 0;
SPARSE_RRT = 1;

% Initialize variables.
if(AStar)
    filename = '/home/adam/Dropbox/Manipulation/MotionPlanning/AStar/Solution.txt';
%     filename = 'C:\Users\Adam\Dropbox\Manipulation\MotionPlanning\AStar\Solution.txt';
elseif(RGRRT)
    filename = '/home/adam/Dropbox/Manipulation/MotionPlanning/RGRRT/build/Solution.txt';
elseif(SPARSE_RRT)
    filename = '/home/adam/Dropbox/Manipulation/MotionPlanning/SPARSE-RRT/build/Solution.txt';
    %filename = 'C:\Users\Adam\Dropbox\Manipulation\MotionPlanning\SPARSE-RRT\build\Solution.txt';
end
delimiter = ' ';
startRow = 2;

%
formatSpec = '%f%f%f%f%f%f%f%f%f%[^\n\r]';

% Open the text file.
fileID = fopen(filename,'r');
% Read columns of data according to format string.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);

% Close the text file.
fclose(fileID);

% Allocate imported array to column variable names
x = dataArray{:, 1};
xd = dataArray{:, 2};
y = dataArray{:, 3};
yd = dataArray{:, 4};
th = dataArray{:, 5};
thd = dataArray{:, 6};
s = dataArray{:, 7};
Fn = dataArray{:, 8};
Ff = dataArray{:, 9};

dt = 0.01;
t = (0:(length(x) - 1))*dt;

%Animate
params = ParametersFunction();
s = s+params.lo;
figure;
set(gca,'fontname','Bitstream Charter','fontsize',18);
hold on
xlim([-0.8 0.1])
ylim([-0.45 0.45])
axis square
%     axis off
%bottom left, bottom right, top right, top left
% xGoal = -0.3; thGoal = pi; yGoal = 0.0;
xGoal = -0.3; yGoal = 0.1; thGoal = 0.4115;
xverts = [xGoal - params.lo*cos(thGoal) + params.wo*sin(thGoal); ...
    xGoal + params.lo*cos(thGoal) + params.wo*sin(thGoal); ...
    xGoal + params.lo*cos(thGoal) - params.wo*sin(thGoal); ...
    xGoal - params.lo*cos(thGoal) - params.wo*sin(thGoal)];
yverts = [yGoal - params.wo*cos(thGoal) - params.lo*sin(thGoal); ...
    yGoal - params.wo*cos(thGoal) + params.lo*sin(thGoal); ...
    yGoal + params.wo*cos(thGoal) + params.lo*sin(thGoal);
    yGoal + params.wo*cos(thGoal) - params.lo*sin(thGoal)];
goal = patch(xverts,yverts,'g');
set(goal,'EdgeColor','g','LineWidth',2);
set(goal,'FaceColor',[1 1 1]);

xverts = [x(1) - params.lo*cos(th(1)) + params.wo*sin(th(1)); ...
    x(1) + params.lo*cos(th(1)) + params.wo*sin(th(1)); ...
    x(1) + params.lo*cos(th(1)) - params.wo*sin(th(1));
    x(1) - params.lo*cos(th(1)) - params.wo*sin(th(1))];
yverts = [y(1) - params.wo*cos(th(1)) - params.lo*sin(th(1)); ...
    y(1) - params.wo*cos(th(1)) + params.lo*sin(th(1)); ...
    y(1) + params.wo*cos(th(1)) + params.lo*sin(th(1));
    y(1) + params.wo*cos(th(1)) - params.lo*sin(th(1))];
contactForceBase = [xverts(2) + s(1)*cos(th(1)); yverts(2) + s(1)* ...
    sin(th(1))];
obj = patch(xverts,yverts,'b');
set(obj,'EdgeColor','b','LineWidth',3);
set(obj,'FaceColor',[1 1 1]);
coneLength = 0.05;
%Do friction cones also
mu = 1.0;
beta = atan(mu);
base1 = [xverts(1); yverts(1)];
left1 = base1 + coneLength*[cos(th(1) + pi/2 + beta); sin(th(1) + pi/2 + ...
    beta)];
right1 = base1 + coneLength*[cos(th(1) + pi/2 - beta); sin(th(1) + pi/2 ...
    - beta)];
base2 = [xverts(2); yverts(2)];
left2 = base2 + coneLength*[cos(th(1) + pi/2 + beta); sin(th(1) + pi/2 + ...
    beta)];
right2 = base2 + coneLength*[cos(th(1) + pi/2 - beta); sin(th(1) + pi/2 ...
    - beta)];
forceMag = coneLength; %norm([Ff(1) Fn(1)]);
forceAngle = th(1) + pi/2  - atan2(Ff(1),Fn(1));
contactForceEnd = contactForceBase + forceMag*[cos(forceAngle); ...
    sin(forceAngle)];
leftLine1 = line([base1(1) left1(1)], [base1(2), left1(2)],'Color','k', ...
    'LineWidth',2);
rightLine1 = line([base1(1) right1(1)],[base1(2), right1(2)],'Color','k', ...
    'LineWidth',2);
leftLine2 = line([base2(1) left2(1)], [base2(2), left2(2)],'Color','k', ...
    'LineWidth',2);
rightLine2 = line([base2(1) right2(1)],[base2(2), right2(2)],'Color','k', ...
    'LineWidth',2);
contactForceLine = line([contactForceBase(1), contactForceEnd(1)], ...
    [contactForceBase(2), contactForceEnd(2)],'Color','r', ...
    'LineWidth',2);
M(length(t)) = struct('cdata',[],'colormap',[]);
j = 1;

if(backwards_int)
    x = flip(x);
    th = flip(th);
    y = flip(y);
    s = flip(s);
    Ff = flip(Ff);
    Fn = flip(Fn);
end
for i=1:length(t)
    xverts = [x(i) - params.lo*cos(th(i)) + params.wo*sin(th(i)); ...
        x(i) + params.lo*cos(th(i)) + params.wo*sin(th(i)); ...
        x(i) + params.lo*cos(th(i)) - params.wo*sin(th(i));
        x(i) - params.lo*cos(th(i)) - params.wo*sin(th(i))];
    yverts = [y(i) - params.wo*cos(th(i)) - params.lo*sin(th(i)); ...
        y(i) - params.wo*cos(th(i)) + params.lo*sin(th(i)); ...
        y(i) + params.wo*cos(th(i)) + params.lo*sin(th(i));
        y(i) + params.wo*cos(th(i)) - params.lo*sin(th(i))];
    set(obj,'XData',xverts,'YData',yverts);
    contactForceBase = [xverts(1) + s(i)*cos(th(i)); yverts(1) + s(i)* ...
            sin(th(i))];
    mu = params.mu;
    beta = atan(mu);
    base1 = [xverts(1); yverts(1)];
    left1 = base1 + coneLength*[cos(th(i) + pi/2 + beta); sin(th(i) + pi/2 + ...
        beta)];
    right1 = base1 + coneLength*[cos(th(i) + pi/2 - beta); sin(th(i) + pi/2 ...
        - beta)];
    set(leftLine1,'XData',[base1(1) left1(1)], ...
        'YData', [base1(2), left1(2)]);
    set(rightLine1,'XData',[base1(1) right1(1)], ...
        'YData', [base1(2), right1(2)]);
    base2 = [xverts(2); yverts(2)];
    left2 = base2 + coneLength*[cos(th(i) + pi/2 + beta); sin(th(i) + pi/2 + ...
        beta)];
    right2 = base2 + coneLength*[cos(th(i) + pi/2 - beta); sin(th(i) + pi/2 ...
        - beta)];
    set(leftLine2,'XData',[base2(1) left2(1)], ...
        'YData', [base2(2), left2(2)]);
    set(rightLine2,'XData',[base2(1) right2(1)], ...
        'YData', [base2(2), right2(2)]);
    forceMag = coneLength; 
    forceAngle = th(i) + pi/2 - atan2(Ff(i),Fn(i));
    contactForceEnd = contactForceBase + forceMag*[cos(forceAngle); ...
        sin(forceAngle)];
    set(contactForceLine,'XData', ...
        [contactForceBase(1), contactForceEnd(1)], ...
        'YData', [contactForceBase(2), contactForceEnd(2)]);
    M(i) = getframe;
end
movie(M,2,1000)
if(saveMovie)
    writerObj = VideoWriter('Sol-13-1-2014-095-1ms-20k.avi');
    set(writerObj,'FrameRate',1000);
    open(writerObj);
    writeVideo(writerObj,M);
    close(writerObj);
end