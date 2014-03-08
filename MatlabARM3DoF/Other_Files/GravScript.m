%Used to analyze data from GravityTest.m

clear y1 t1 y2 t2 y3 t3 y4 t4 y5 t5 y6 t6 y7 t7 y8 t8 y9 t9 y10 t10 y11 t11 y12 t12

%up
y1 = posYFixed(463:718);
t1 = tFixed(463:718);

%down 
y2 = posYFixed(887:1182);
t2 = tFixed(887:1182);

%up&down 
y3 = posYFixed(1377:1643);
t3 = tFixed(1377:1643);

%up&down (1785:2055)
y4 = posYFixed(1785:2055);
t4 = tFixed(1785:2055);

%up&down (2221:2488)
y5 = posYFixed(2221:2488);
t5 = tFixed(2221:2488);

%up&down (2654:2941)
y6 = posYFixed(2654:2941);
t6 = tFixed(2654:2941);

%up&down (3002:3256)
y7 = posYFixed(3002:3256);
t7 = tFixed(3002:3256);

%
y8 = posYFixed(3444:3761);
t8 = tFixed(3444:3761);

%
y9 = posYFixed(3975:4244);
t9 = tFixed(3975:4244);

%
y10 = posYFixed(5436:5700);
t10 = tFixed(5436:5700);

%
y11 = posYFixed(5784:6038);
t11 = tFixed(5784:6038);

%
y12 = posYFixed(6232:6435);
t12 = tFixed(6232:6435);

% %"Fix" the data, remove duplicates
% j = 1;
% y1Fixed(j) = y1(j);
% t1Fixed(j) = t1(j);
% for i = 2:length(y1)
%     if(y1Fixed(j) ~= y1(i))
%         j = j+1;
%         y1Fixed(j) = y1(i);
%         t1Fixed(j) = t1(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y1Fixed = y1Fixed(2:end);
% t1Fixed = t1Fixed(2:end);
% 
% %"Fix" the data, remove duplicates
% j = 1;
% y2Fixed(j) = y2(j);
% t2Fixed(j) = t2(j);
% for i = 2:length(y2)
%     if(y2Fixed(j) ~= y2(i))
%         j = j+1;
%         y2Fixed(j) = y2(i);
%         t2Fixed(j) = t2(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y2Fixed = y2Fixed(2:end);
% t2Fixed = t2Fixed(2:end);
% 
% %"Fix" the data, remove duplicates
% j = 1;
% y3Fixed(j) = y3(j);
% t3Fixed(j) = t3(j);
% for i = 2:length(y3)
%     if(y3Fixed(j) ~= y3(i))
%         j = j+1;
%         y3Fixed(j) = y3(i);
%         t3Fixed(j) = t3(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y3Fixed = y3Fixed(2:end);
% t3Fixed = t3Fixed(2:end);
% 
% %"Fix" the data, remove duplicates
% j = 1;
% y4Fixed(j) = y4(j);
% t4Fixed(j) = t4(j);
% for i = 2:length(y1)
%     if(y4Fixed(j) ~= y4(i))
%         j = j+1;
%         y4Fixed(j) = y4(i);
%         t4Fixed(j) = t4(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y4Fixed = y4Fixed(2:end);
% t4Fixed = t4Fixed(2:end);
% 
% %"Fix" the data, remove duplicates
% j = 1;
% y5Fixed(j) = y5(j);
% t5Fixed(j) = t5(j);
% for i = 2:length(y5)
%     if(y5Fixed(j) ~= y5(i))
%         j = j+1;
%         y5Fixed(j) = y5(i);
%         t5Fixed(j) = t5(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y5Fixed = y5Fixed(2:end);
% t5Fixed = t5Fixed(2:end);
% 
% %"Fix" the data, remove duplicates
% j = 1;
% y6Fixed(j) = y6(j);
% t6Fixed(j) = t6(j);
% for i = 2:length(y6)
%     if(y6Fixed(j) ~= y6(i))
%         j = j+1;
%         y6Fixed(j) = y6(i);
%         t6Fixed(j) = t6(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y6Fixed = y6Fixed(2:end);
% t6Fixed = t6Fixed(2:end);
% 
% %"Fix" the data, remove duplicates
% j = 1;
% y7Fixed(j) = y7(j);
% t7Fixed(j) = t7(j);
% for i = 2:length(y7)
%     if(y7Fixed(j) ~= y7(i))
%         j = j+1;
%         y7Fixed(j) = y7(i);
%         t7Fixed(j) = t7(i);
%     end
% end
% %Remove first data point, might be screwed up in terms of timing
% y7Fixed = y7Fixed(2:end);
% t7Fixed = t7Fixed(2:end);

%Calculate velocities and accelerations
y1Vel = derivative(y1)./derivative(t1);
y2Vel = derivative(y2)./derivative(t2);
y3Vel = derivative(y3)./derivative(t3);
y4Vel = derivative(y4)./derivative(t4);
y5Vel = derivative(y5)./derivative(t5);
y6Vel = derivative(y6)./derivative(t6);
y7Vel = derivative(y7)./derivative(t7);
y8Vel = derivative(y8)./derivative(t8);
y9Vel = derivative(y9)./derivative(t9);
y10Vel = derivative(y10)./derivative(t10);
y11Vel = derivative(y11)./derivative(t11);
y12Vel = derivative(y12)./derivative(t12);
y1Acc = derivative(y1Vel)./derivative(t1);
y2Acc = derivative(y2Vel)./derivative(t2);
y3Acc = derivative(y3Vel)./derivative(t3);
y4Acc = derivative(y4Vel)./derivative(t4);
y5Acc = derivative(y5Vel)./derivative(t5);
y6Acc = derivative(y6Vel)./derivative(t6);
y7Acc = derivative(y7Vel)./derivative(t7);
y8Acc = derivative(y8Vel)./derivative(t8);
y9Acc = derivative(y9Vel)./derivative(t9);
y10Acc = derivative(y10Vel)./derivative(t10);
y11Acc = derivative(y11Vel)./derivative(t11);
y12Acc = derivative(y12Vel)./derivative(t12);

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
p1 = polyfit(t1,y1,2);
p2 = polyfit(t2,y2,2);
p3 = polyfit(t3,y3,2);
p4 = polyfit(t4,y4,2);
p5 = polyfit(t5,y5,2);
p6 = polyfit(t6,y6,2);
p7 = polyfit(t7,y7,2);
p8 = polyfit(t8,y8,2);
p9 = polyfit(t9,y9,2);
p10 = polyfit(t10,y10,2);
p11 = polyfit(t11,y11,2);
p12 = polyfit(t12,y12,2);
p1(1)*2
p2(1)*2
p3(1)*2
p4(1)*2
p5(1)*2
p6(1)*2
p7(1)*2
p8(1)*2
p9(1)*2
p10(1)*2
p11(1)*2
p12(1)*2
mean([p1(1)*2 p2(1)*2 p3(1)*2 p4(1)*2 p5(1)*2 p6(1)*2 p7(1)*2 p8(1)*2 ...
    p9(1)*2 p10(1)*2 p11(1)*2 p12(1)*2])

%1
figure
plot(t1,y1,'r.',t1,polyval(p1,t1),'-b');
title('1')
xlabel('Time (s)'); ylabel('m')
%2
figure
plot(t2,y2,'r.',t2,polyval(p2,t2),'-b');
title('2')
xlabel('Time (s)'); ylabel('m')
%3
figure
plot(t3,y3,'r.',t3,polyval(p3,t3),'-b');
title('3')
xlabel('Time (s)'); ylabel('m')
%4
figure
plot(t4,y4,'r.',t4,polyval(p4,t4),'-b');
title('4')
xlabel('Time (s)'); ylabel('m')
%5
figure
plot(t5,y5,'r.',t5,polyval(p5,t5),'-b');
title('5')
xlabel('Time (s)'); ylabel('m')
%6
figure
plot(t6,y6,'r.',t6,polyval(p6,t6),'-b');
title('6')
xlabel('Time (s)'); ylabel('m')
%7
figure
plot(t7,y7,'r.',t7,polyval(p7,t7),'-b');
title('7')
xlabel('Time (s)'); ylabel('m')
%8
figure
plot(t8,y8,'r.',t8,polyval(p8,t8),'-b');
title('8')
xlabel('Time (s)'); ylabel('m')
%9
figure
plot(t9,y9,'r.',t9,polyval(p9,t9),'-b');
title('9')
xlabel('Time (s)'); ylabel('m')
%10
figure
plot(t10,y10,'r.',t10,polyval(p10,t10),'-b');
title('10')
xlabel('Time (s)'); ylabel('m')
%11
figure
plot(t11,y11,'r.',t11,polyval(p11,t11),'-b');
title('11')
xlabel('Time (s)'); ylabel('m')
%12
figure
plot(t12,y12,'r.',t12,polyval(p12,t12),'-b');
title('12')
xlabel('Time (s)'); ylabel('m')