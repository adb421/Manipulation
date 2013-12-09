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
j = 1;
t = 0:0.001:29.999;
posXFixed(j) = posX(1); posYFixed(j) = posY(1);
posThFixed(j) = posTh(1); tFixed(j) = t(1);
j = j+1;
for i = 2:length(posY)
    if(posY(i) ~= posY(i-1))
        posXFixed(j) = posX(i);
        posYFixed(j) = posY(i);
        posThFixed(j) = posTh(i);
        tFixed(j) = t(i);
        j = j+1;
    end
end

figure;
plot(posXFixed);
figure;
plot(posYFixed);
figure;
plot(posThFixed);
pc104.killProgram();