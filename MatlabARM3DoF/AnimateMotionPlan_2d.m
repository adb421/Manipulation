clear
% Animates solutions found by AStar in the forms of "Solution.txt"

backwards_int = 0;
AStar = 1;
RGRRT = 0;
SPARSE_RRT = 0;

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
Fn = dataArray{:, 3};

dt = 0.005;
t = (0:(length(x) - 1))*dt;

%Animate
params = ParametersFunction();
figure;
set(gca,'fontname','Bitstream Charter','fontsize',18);
hold on
xlim([-8 25]*0.0254)
ylim([-4 4])
axis square
%bottom left, bottom right, top right, top left

xInit = 0.0; xDotInit = 0.0;
xGoal = 0.2; xDotGoal = 0.5;

if(backwards_int)
    x = flip(x);
    xd = flip(xd);
    Fn = flip(Fn);
end

plot(x,xd,'.-b');
hold on
plot([xInit, xGoal],[xDotInit, xDotGoal], '--r');
hold off
xlabel('Y Position (m)')
ylabel('YDot Velocity (m/s)')