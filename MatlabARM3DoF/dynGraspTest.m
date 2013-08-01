% %Script to do dynamic grasp testing
% %Adam Barber
% %Written 3/1/2013
% 
% clear all
% close all
% 
% set up parameters
% params = ParametersFunction();
% dt = 0.001;
% 
% %Connect to PC104
% port_in = 3100;
% port_out = 3490;
% 
% %Check if ports are already opened
% while ~isempty(instrfind)
%     fclose(instrfind);
%     delete(instrfind);
% end
% 
% disp('Attempting to connect');
% IPin = tcpip('192.168.1.50', port_in);
% IPout = tcpip('192.168.1.50', port_out);
% set(IPin,'ByteOrder','littleEndian');
% set(IPin,'InputBufferSize',4095);
% set(IPout,'ByteOrder','littleEndian');
% set(IPout,'OutputBufferSize',4095);
% 
% fopen(IPin)
% fopen(IPout)
% pause(1);
%%
dt = 0.001;
params = ParametersFunction();
%Send controls
cmd = 6;
fwrite(IPout,cmd,'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'CONTROL')))
    disp('Problem updating gains');
    disp(text);
    return;
end
disp('Sending control gains');
for i = 1:3
    fwrite(IPout, params.kp(i),'double');
end
for i = 1:3
    fwrite(IPout, params.kd(i),'double');
end
for i = 1:3
    fwrite(IPout, params.ki(i),'double');
end
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'GAINSUPDATED')))
    disp('Gains not updated');
    return;
end
disp('Connected');

%%Get a trajectory
[xObjTraj, yObjTraj, thObjTraj, t] = GraspLoopTraj();
T = t(end);
num_pts = length(t);

%%
% Reset encoder values based on camera
cmd = 10;
fwrite(IPout, cmd, 'int32');
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'ENCRESET')))
    disp('Encoders not reset')
    disp(text)
    return
end
disp('Encoder counts reset');

%%
% Send home positions
cmd = 7;
fwrite(IPout, cmd, 'int32');
fwrite(IPout, xObjTraj(1),'double');
fwrite(IPout, yObjTraj(1),'double');
fwrite(IPout, thObjTraj(1),'double');
% fwrite(IPout, -0.25,'double');
% fwrite(IPout, -0.25,'double');
% fwrite(IPout, 0.05,'double');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'HOMEUPDATED')))
    disp('Home not updated')
    disp(text)
    return
end
disp('Home positions sent')
disp('unpause to go home')
pause
%%
%Go home!
cmd = 4;
fwrite(IPout,cmd,'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'HOME')))
    disp('Not going home???')
    disp(text)
    return
end
disp('Going home!')

% disp('Unpause to ESTOP!')
% pause
% cmd = 9;
% fwrite(IPout, cmd, 'int32');
% text = fscanf(IPin,'%s');
% while(isempty(strfind(text, 'STOP')))
%     fwrite(IPout, 'int32');
%     text = fscanf(IPin,'%s');
% end
% fscanf(IPin,'%lu',[1 1]);
% fscanf(IPin,'%s');
% disp('Stopped and reset!');

%%
%Send that traj
cmd = 1;
fwrite(IPout, cmd, 'int32');
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'GETTRAJ')))
    disp('No traj command');
    disp(text)
    return
end

%Write numpts
fwrite(IPout, num_pts, 'int32');
reading = fread(IPin, 1, 'int32');
if(isempty(reading) || reading ~= num_pts)
    disp('No cmd or right # of data pts')
    disp(reading)
    return
end

%Check for memory allocation
text = fscanf(IPin, '%s');
if(~isempty(strfind(text,'MEMFAIL')))
    disp('Not enough memory :(')
    disp(text)
    return
end

%Send manipulator trajectories
cmd = 2;
fwrite(IPout, cmd, 'int32');
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'SENDDATA')))
    disp('PC104 doesnt want data')
    disp(text)
    return
end

%X
for i = 1:num_pts
    fwrite(IPout, xObjTraj(i),'double');
end
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'DONETRAJ1')))
    disp('Didnt get first traj')
    disp(text)
    return
end
disp('Sent traj 1')
%Y
for i = 1:num_pts
    fwrite(IPout, yObjTraj(i),'double');
end
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'DONETRAJ2')))
    disp('Didnt get second traj')
    disp(text)
    return
end
disp('Sent traj 2')
%Theta
for i = 1:num_pts
    fwrite(IPout, thObjTraj(i),'double');
end
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'DONETRAJ3')))
    disp('Didnt get third traj')
    disp(text)
end
disp('Sent traj 3')
disp('Traj sent')
disp('Unpause for traj go')
pause
pause(15)
%%
%Execute!
cmd = 5;
fwrite(IPout, cmd, 'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'EXECUTE')))
    disp('No execute?')
    disp(text)
    return;
end
disp(['Executing, please wait ' num2str(t(end)) 'seconds']);
pause(t(end) + 0.2)

%%
%Get yo' data
%START
th1Act = zeros(1,num_pts);
th2Act = zeros(1,num_pts);
th3Act = zeros(1,num_pts);
controlVals1 = zeros(1,num_pts);
controlVals2 = zeros(1,num_pts);
controlVals3 = zeros(1,num_pts);
xObjCam = zeros(1,num_pts);
yObjCam = zeros(1,num_pts);
thObjCam = zeros(1,num_pts);
loopTimes = zeros(1,num_pts);
objX = zeros(1,num_pts);
objY = zeros(1,num_pts);
objTh = zeros(1,num_pts);
%END
%RESET
cmd = 3;
fwrite(IPout, cmd, 'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'START')))
    disp('No get data?')
    disp(text)
    return
end
for i = 1:num_pts
    th1Act(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    th2Act(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    th3Act(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    controlVals1(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    controlVals2(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    controlVals3(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    xObjCam(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    yObjCam(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    thObjCam(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    loopTimes(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    objX(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    objY(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    objTh(i) = fread(IPin,1,'double');
end
text = fscanf(IPin,'%s'); %end
while(~strfind(text,'RESET'))
    text = fscanf(IPin,'%s'); %Reset
end
disp('Done!')
disp(text)

%%
%PLOTS
figure
plot(t,objX,'.r',t,xObjTraj,'-b', t,xObjCam,'.g');
title('Object x position, red-actual(enc), green-actual(cam), blue-desired')
xlabel('time (s)')
ylabel('Position (m)')
figure
plot(t,objY,'.r',t,yObjTraj,'-b', t,yObjCam,'.g');
title('Object y position, red-actual, blue-desired')
xlabel('time (s)')
ylabel('Position (m)')
figure
plot(t,objTh,'.r',t,thObjTraj,'-b', t,yObjCam,'.g');
title('Object th position, red-actual, blue-desired')
xlabel('time (s)')
ylabel('Position (m)')

figure
plot(t,loopTimes);
title('Time for control loop to execute')
xlabel('Time (s)');
ylabel('Loop execution time (ms)');

figure
xManip = -params.L1*cos(th1Act) - params.L2*cos(th1Act+th2Act);
yManip = -params.L1*sin(th1Act) - params.L2*sin(th1Act+th2Act);
plot(xManip,yManip,'r',xObjTraj,yObjTraj,'b',objX,objY,'g')
figure
plot(t,th1Act+th2Act+th3Act,'r',t, thObjTraj,'b', t, objTh,'g');