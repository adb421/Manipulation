clear
% Plot full trees
AStar = 0;
RGRRT = 0;
SPARSE_RRT = 1;

% Initialize variables.
if(AStar)
    filename = '/home/adam/Dropbox/Manipulation/MotionPlanning/AStar/Solution.txt';
elseif(RGRRT)
    filename = '/home/adam/Dropbox/Manipulation/MotionPlanning/RGRRT/build/Solution.txt';
elseif(SPARSE_RRT)
    filename = '/home/adam/Dropbox/Manipulation/MotionPlanning/SPARSE-RRT/build/Tree.txt';
%     filename = 'C:\Users\Adam\Dropbox\Manipulation\MotionPlanning\SPARSE-RRT\build\Tree.txt';
end
delimiter = ' ';
startRow = 2;

initState = [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0];
goalState = [-0.3, -0.0525, 0.1, 0.0245, 0.4115, 0.8511];

%
formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

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
P = dataArray{:,10};

%Create x,y,th tuples based on parent-child relationship
xFlip = fliplr(x);
yFlip = fliplr(y);
thFlip = fliplr(th);
pFlip = fliplr(P) + 1;
xdFlip = fliplr(xd);
ydFlip = fliplr(yd);
thdFlip = fliplr(thd);
figure(1);
plot3([initState(1) goalState(1)], [initState(3) goalState(3)], [initState(5), goalState(5)],'r');
xlabel('x (m)');
ylabel('y (m)');
zlabel('th (rad)');
hold on;
for j = 1:length(xFlip)
    if(pFlip(j) ~= 0)
        plot3([xFlip(j) x(pFlip(j))], [yFlip(j) y(pFlip(j))], [thFlip(j) th(pFlip(j))],'b');
    end
    hold on;
end

figure(2);
plot([initState(1) goalState(1)], [initState(2) goalState(2)],'r');
hold on;
xlabel('x (m)');
ylabel('xdot (m/s)');
for j = 1:length(xFlip)
    if(pFlip(j) ~= 0)
        plot([xFlip(j) x(pFlip(j))], [xdFlip(j), xd(pFlip(j))],'b');
    end
    hold on
end

figure(3);
plot([initState(3) goalState(3)], [initState(4) goalState(4)],'r');
hold on;
xlabel('y (m)');
ylabel('ydot (m/s)');
for j = 1:length(xFlip)
    if(pFlip(j) ~= 0)
        plot([yFlip(j) y(pFlip(j))], [ydFlip(j), yd(pFlip(j))],'b');
    end
    hold on
end

figure(4);
plot([initState(5) goalState(5)], [initState(6) goalState(6)],'r');
hold on;
xlabel('th (rad)');
ylabel('thdot (rad/s)');
for j = 1:length(xFlip)
    if(pFlip(j) ~= 0)
        plot([thFlip(j) th(pFlip(j))], [thdFlip(j), thd(pFlip(j))],'b');
    end
    hold on
end
