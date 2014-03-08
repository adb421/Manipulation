% This test script was originally written to calculate gravity on the air table
% and be able to figure out the inclination angle as well. This framework could be used to do anything involving camera measurements and no actuation.

% This script assumes NO_CONTROL (0) is the execution mode.

%Whole point is to analyze gravity.

%Record data for 30 seconds.
t = 0:0.001:30;
num_pts = length(t);

%These are pretty much always called
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.allocateTraj(num_pts);

disp('unpause to start recording data');
pause
%Go traj in this case is basically just record data as the control mode is set to NO_CONTROL (0)
pc104.goTraj();
pause(30);
pc104.getTrajData();
%We got the data, so kill the program on the PC104
pc104.killProgram();


posX = pc104.objPosX;
posY = pc104.objPosY;
posTh = pc104.objPosTh;
j = 1;
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
