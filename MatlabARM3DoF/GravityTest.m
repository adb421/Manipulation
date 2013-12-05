%Empty trajcetory, already changed the pc104 code to run with no control and
%only send back object positions. Should work faster. Don't even need to send
%a trajectory

%Whole point is to analyze gravity.

%Record data for 30 seconds.
num_pts = 30/0.001;

pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.allocateTraj(num_pts);

disp('unpause to start recording data');
pause
pc104.goTraj();
pause(30);
pc104.getTrajData();
posX = pc104.objPosX;
posY = pc104.objPosY;
posTh = pc104.objPosTh;
t = 0:0.001:29.999;
figure;
plot(t,posX);
figure;
plot(t,posY);
figure;
plot(t,posTh);
pc104.killProgram();