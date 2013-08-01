clear y1 t1 y2 t2 y3 t3 y4 t4 y5 t5 y6 t6 y7 t7
%up
y1 = posY(3030:3500);
t1 = t(3030:3500);

%down
y2 = posY(3669:4190);
t2 = t(3669:4190);

%up&down
y3 = posY(4915:6000);
t3 = t(4915:6000);

%up&down
y4 = posY(9638:10860);
t4 = t(9638:10860);

%up&down
y5 = posY(11620:12500);
t5 = t(11620:12500);

%up&down
y6 = posY(13750:14600);
t6 = t(13750:14600);

%up&down
y7 = posY(27370:27910);
t7 = t(27370:27910);

%"Fix" the data, remove duplicates
j = 1;
y1Fixed(j) = y1(j);
t1Fixed(j) = t1(j);
for i = 2:length(y1)
    if(y1Fixed(j) ~= y1(i))
        j = j+1;
        y1Fixed(j) = y1(i);
        t1Fixed(j) = t1(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y1Fixed = y1Fixed(2:end);
t1Fixed = t1Fixed(2:end);

%"Fix" the data, remove duplicates
j = 1;
y2Fixed(j) = y2(j);
t2Fixed(j) = t2(j);
for i = 2:length(y2)
    if(y2Fixed(j) ~= y2(i))
        j = j+1;
        y2Fixed(j) = y2(i);
        t2Fixed(j) = t2(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y2Fixed = y2Fixed(2:end);
t2Fixed = t2Fixed(2:end);

%"Fix" the data, remove duplicates
j = 1;
y3Fixed(j) = y3(j);
t3Fixed(j) = t3(j);
for i = 2:length(y3)
    if(y3Fixed(j) ~= y3(i))
        j = j+1;
        y3Fixed(j) = y3(i);
        t3Fixed(j) = t3(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y3Fixed = y3Fixed(2:end);
t3Fixed = t3Fixed(2:end);

%"Fix" the data, remove duplicates
j = 1;
y4Fixed(j) = y4(j);
t4Fixed(j) = t4(j);
for i = 2:length(y1)
    if(y4Fixed(j) ~= y4(i))
        j = j+1;
        y4Fixed(j) = y4(i);
        t4Fixed(j) = t4(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y4Fixed = y4Fixed(2:end);
t4Fixed = t4Fixed(2:end);

%"Fix" the data, remove duplicates
j = 1;
y5Fixed(j) = y5(j);
t5Fixed(j) = t5(j);
for i = 2:length(y5)
    if(y5Fixed(j) ~= y5(i))
        j = j+1;
        y5Fixed(j) = y5(i);
        t5Fixed(j) = t5(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y5Fixed = y5Fixed(2:end);
t5Fixed = t5Fixed(2:end);

%"Fix" the data, remove duplicates
j = 1;
y6Fixed(j) = y6(j);
t6Fixed(j) = t6(j);
for i = 2:length(y6)
    if(y6Fixed(j) ~= y6(i))
        j = j+1;
        y6Fixed(j) = y6(i);
        t6Fixed(j) = t6(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y6Fixed = y6Fixed(2:end);
t6Fixed = t6Fixed(2:end);

%"Fix" the data, remove duplicates
j = 1;
y7Fixed(j) = y7(j);
t7Fixed(j) = t7(j);
for i = 2:length(y7)
    if(y7Fixed(j) ~= y7(i))
        j = j+1;
        y7Fixed(j) = y7(i);
        t7Fixed(j) = t7(i);
    end
end
%Remove first data point, might be screwed up in terms of timing
y7Fixed = y7Fixed(2:end);
t7Fixed = t7Fixed(2:end);

%Calculate velocities and accelerations
y1Vel = derivative(y1Fixed)./derivative(t1Fixed);
y2Vel = derivative(y2Fixed)./derivative(t2Fixed);
y3Vel = derivative(y3Fixed)./derivative(t3Fixed);
y4Vel = derivative(y4Fixed)./derivative(t4Fixed);
y5Vel = derivative(y5Fixed)./derivative(t5Fixed);
y6Vel = derivative(y6Fixed)./derivative(t6Fixed);
y7Vel = derivative(y7Fixed)./derivative(t7Fixed);
y1Acc = derivative(y1Vel)./derivative(t1Fixed);
y2Acc = derivative(y2Vel)./derivative(t2Fixed);
y3Acc = derivative(y3Vel)./derivative(t3Fixed);
y4Acc = derivative(y4Vel)./derivative(t4Fixed);
y5Acc = derivative(y5Vel)./derivative(t5Fixed);
y6Acc = derivative(y6Vel)./derivative(t6Fixed);
y7Acc = derivative(y7Vel)./derivative(t7Fixed);

% %1
% figure
% subplot(3,1,1)
% plot(t1Fixed,y1Fixed);
% title('Going up 1');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t1Fixed,y1Vel);
% title('Vel 1');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t1Fixed,y1Acc)
% title('Accel 1');
% xlabel('Time'); ylabel('m/s^2');
% %2
% figure
% subplot(3,1,1)
% plot(t2Fixed,y2Fixed);
% title('Going up 2');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t2Fixed,y2Vel);
% title('Vel 2');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t2Fixed,y2Acc)
% title('Accel 2');
% xlabel('Time'); ylabel('m/s^2');
% %3
% figure
% subplot(3,1,1)
% plot(t3Fixed,y3Fixed);
% title('Going up 3');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t3Fixed,y3Vel);
% title('Vel 3');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t3Fixed,y3Acc)
% title('Accel 3');
% xlabel('Time'); ylabel('m/s^2');
% %4
% figure
% subplot(3,1,1)
% plot(t4Fixed,y4Fixed);
% title('Going up 4');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t4Fixed,y4Vel);
% title('Vel 4');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t4Fixed,y4Acc)
% title('Accel 4');
% xlabel('Time'); ylabel('m/s^2');
% %5
% figure
% subplot(3,1,1)
% plot(t5Fixed,y5Fixed);
% title('Going up 5');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t5Fixed,y5Vel);
% title('Vel 5');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t5Fixed,y5Acc)
% title('Accel 5');
% xlabel('Time'); ylabel('m/s^2');
% %6
% figure
% subplot(3,1,1)
% plot(t6Fixed,y6Fixed);
% title('Going up 6');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t6Fixed,y6Vel);
% title('Vel 6');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t6Fixed,y6Acc)
% title('Accel 6');
% xlabel('Time'); ylabel('m/s^2');
% %7
% figure
% subplot(3,1,1)
% plot(t7Fixed,y7Fixed);
% title('Going up 7');
% xlabel('Time'); ylabel('meters');
% subplot(3,1,2)
% plot(t7Fixed,y7Vel);
% title('Vel 7');
% xlabel('Time'); ylabel('m/s');
% subplot(3,1,3)
% plot(t7Fixed,y7Acc)
% title('Accel 7');
% xlabel('Time'); ylabel('m/s^2');

% mean(y1Acc)
% mean(y2Acc)
% mean(y3Acc)
% mean(y4Acc)
% mean(y5Acc)
% mean(y6Acc)
% mean(y7Acc)
% mean([y2Acc y3Acc y4Acc y5Acc y6Acc y7Acc])
p1 = polyfit(t1Fixed,y1Fixed,2);
p2 = polyfit(t2Fixed,y2Fixed,2);
p3 = polyfit(t3Fixed,y3Fixed,2);
p4 = polyfit(t4Fixed,y4Fixed,2);
p5 = polyfit(t5Fixed,y5Fixed,2);
p6 = polyfit(t6Fixed,y6Fixed,2);
p7 = polyfit(t7Fixed,y7Fixed,2);
p1(1)*2
p2(1)*2
p3(1)*2
p4(1)*2
p5(1)*2
p6(1)*2
p7(1)*2
mean([p1(1)*2 p2(1)*2 p3(1)*2 p4(1)*2 p5(1)*2 p6(1)*2 p7(1)*2])

%1
figure
plot(t1Fixed,y1Fixed,'r.',t1Fixed,polyval(p1,t1Fixed),'-b');
title('1')
xlabel('Time (s)'); ylabel('m')
%2
figure
plot(t2Fixed,y2Fixed,'r.',t2Fixed,polyval(p2,t2Fixed),'-b');
title('2')
xlabel('Time (s)'); ylabel('m')
%3
figure
plot(t3Fixed,y3Fixed,'r.',t3Fixed,polyval(p3,t3Fixed),'-b');
title('3')
xlabel('Time (s)'); ylabel('m')
%4
figure
plot(t4Fixed,y4Fixed,'r.',t4Fixed,polyval(p4,t4Fixed),'-b');
title('4')
xlabel('Time (s)'); ylabel('m')
%5
figure
plot(t5Fixed,y5Fixed,'r.',t5Fixed,polyval(p5,t5Fixed),'-b');
title('5')
xlabel('Time (s)'); ylabel('m')
%6
figure
plot(t6Fixed,y6Fixed,'r.',t6Fixed,polyval(p6,t6Fixed),'-b');
title('6')
xlabel('Time (s)'); ylabel('m')
%7
figure
plot(t7Fixed,y7Fixed,'r.',t7Fixed,polyval(p7,t7Fixed),'-b');
title('7')
xlabel('Time (s)'); ylabel('m')