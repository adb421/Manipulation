% %Script to test manipulator trajectory following
% %Adam Barber
% %Written 3/4/2013
% 
% clear all
% close all
% 
% %set up parameters
% params = ParametersFunction();
% dt = 0.001;
% 
% %Set up connections to PC104
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
% set(IPin, 'ByteOrder', 'littleEndian');
% set(IPin, 'InputBufferSize', 4095);
% set(IPout, 'ByteOrder', 'littleEndian');
% set(IPout, 'InputBufferSize', 4095);
% 
% fopen(IPin)
% fopen(IPout)
% pause(0.5)

%%
dt = 0.001;
params = ParametersFunction();
%send controls
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

%%
%Get a trajectory
[xManipTraj, yManipTraj, thManipTraj, T] = ManipulatorTrajectory();
t = 0:dt:T;
num_pts = length(t);

%%
%Reset encoder values based on camera
cmd = 10;
fwrite(IPout,cmd,'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'ENCRESET')))
    disp('Encoders not reset')
    disp(text)
    return
end
disp('Encoder counts reset')
%%
%had force contact point calc here, don't need it any more
%send home positions
cmd = 7;
fwrite(IPout,cmd,'int32');
fwrite(IPout,xManipTraj(1),'double');
fwrite(IPout,yManipTraj(1),'double');
fwrite(IPout,thManipTraj(1),'double');
% fwrite(IPout, -0.2, 'double');
% fwrite(IPout, -0.2, 'double');
% fwrite(IPout, 0,'double');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'HOMEUPDATED')))
    disp('Homes not updated')
    disp(text)
    return
end
disp('home positions sent')
disp('unpause to go home')
pause
%%
%Go home!
cmd = 4;
fwrite(IPout,cmd,'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'HOME')))
    disp('Not going home?')
    disp(text)
    return
end
disp('Going home!')

%%
%Send that trajectory
cmd = 1;
fwrite(IPout, cmd, 'int32');
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'GETTRAJ')))
    disp('PC104 didnt get traj command');
    disp(text);
    return;
end

%Write number of data points
fwrite(IPout, num_pts, 'int32');
reading = fread(IPin,1,'int32');
if(isempty(reading) || reading ~= num_pts)
    disp('Didnt get command or right number of points');
    disp(reading);
    return;
end

%Make sure memory was allocated
text = fscanf(IPin, '%s');
if(~isempty(strfind(text,'MEMFAIL')))
    disp('Not enough memory')
    disp(text)
    return
end

%Send manipulator trajectories
cmd = 2;
fwrite(IPout, cmd, 'int32')
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'SENDDATA')))
    disp('PC104 doesnt want data')
    disp(text);
    return
end
%x
for i = 1:num_pts
    fwrite(IPout, xManipTraj(i),'double');
end
text = fscanf(IPin, '%s');
if(isempty(strfind(text,'DONETRAJ1')))
    disp('Didnt get first traj')
    disp(text)
    return
end
disp('Sent first traj')
%y
for i = 1:num_pts
    fwrite(IPout, yManipTraj(i),'double');
end
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'DONETRAJ2')))
    disp('Didnt get second traj')
    disp(text)
    return
end
disp('Sent second traj')
%th
for i = 1:num_pts
    fwrite(IPout, thManipTraj(i),'double');
end
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'DONETRAJ3')))
    disp('Didnt get third traj')
    disp(text)
end
disp('Sent third traj')
disp('Traj sent succesfully')

% disp('Unpause for ESTOP')
disp('Unpause for traj go!')
pause
% cmd = 9;
% fwrite(IPout, cmd, 'int32');
% text = fscanf(IPin,'%s');
% while(isempty(strfind(text, 'STOP')))
%     fwrite(IPin, 'int32');
%     text = fscanf(IPin,'%s');
% end
% fscanf(IPin,'%lu',[1 1]);
% fscanf(IPin,'%s');
% disp('Stopped and reset!');



%%
%execute trajectory!
cmd = 5;
fwrite(IPout, cmd,'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'EXECUTE')))
    disp('No execute?')
    disp(text)
    return;
end
disp(['Executing, please wait ' num2str(t(end)) 'seconds']);
pause(t(end)+0.5)

%%

th1Act = zeros(1,num_pts);
th2Act = zeros(1,num_pts);
th3Act = zeros(1,num_pts);
control1 = zeros(1,num_pts);
control2 = zeros(1,num_pts);
control3 = zeros(1,num_pts);
% loopTimes = zeros(1,num_pts);
xManipCam = zeros(1,num_pts);
yManipCam = zeros(1,num_pts);
% desAccel1 = zeros(1,num_pts);
% desAccel2 = zeros(1,num_pts);
% desAccel3 = zeros(1,num_pts);
xManipEnc = zeros(1,num_pts);
yManipEnc = zeros(1,num_pts);
thManipEnc = zeros(1,num_pts);
%receive trajectory!
cmd = 3;
fwrite(IPout, cmd,'int32');
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'START')))
    disp('Not getting traj data?')
    disp(text)
    return
end
disp(text)
disp('Ready to get traj data')
%Read positions
for i = 1:num_pts
    th1Act(i) = fread(IPin,1,'double');
end
disp('Received joint 1');
for i = 1:num_pts
    th2Act(i) = fread(IPin,1,'double');
end
disp('Received joint 2');
for i = 1:num_pts
    th3Act(i) = fread(IPin,1,'double');
end
disp('Received joint 3');
% text = fscanf(IPin,'%s');
% if(isempty(strfind(text,'POSEND')))
%     disp('Error receiving traj data')
%     disp(text)
%     return
% end
disp('Got all joints, getting controls')
for i = 1:num_pts
    control1(i) = fread(IPin,1,'double');
end
disp('Got control 1')
for i = 1:num_pts
    control2(i) = fread(IPin,1,'double');
end
disp('Got control 2')
for i = 1:num_pts
    control3(i) = fread(IPin,1,'double');
end
disp('Got control 3');
for i = 1:num_pts
    xManipCam(i) = fread(IPin,1,'double');
end
for i = 1:num_pts
    yManipCam(i) = fread(IPin,1,'double');
end
text = fscanf(IPin,'%s');
if(isempty(strfind(text,'END')))
    disp('No loop times?')
    disp(text)
    return
end
disp(text)

fscanf(IPin, '%lu',[1 1]);
text = fscanf(IPin,'%s');
% if(isempty(strfind(text,'RESET')))
%     disp('Error receiving reset')
%     disp(text)
%     return
% end
disp(text)
disp('Done!')

%%
%Plots!

%Calculate x-y from encoders
xManipEnc = -1.0*params.L1*cos(th1Act) - params.L2*cos(th1Act + th2Act);
yManipEnc = -1.0*params.L1*sin(th1Act) - params.L2*sin(th1Act + th2Act);
thManipEnc = th1Act + th2Act + th3Act;

%Plot in x-y space
figure;
plot(xManipCam,yManipCam,'-r',xManipTraj,yManipTraj,'-b',xManipEnc, ...
     yManipEnc,'-g');
axis equal;
title(['Trajectory in x-y space, red - camera, blue - desired, green - ' ...
       'encoders']);
%Plot x,y,th trajs
figure;
plot(t,xManipCam,'-r',t,xManipTraj,'-b',t,xManipEnc,'-g');
title('x traj, red-cam, blue-des,green-enc');
figure;
plot(t,yManipCam,'-r',t,yManipTraj,'-b',t,yManipEnc,'-g');
title('y traj, red-cam, blue-des,green-enc');
figure;
plot(t,thManipTraj,'-b',t,thManipEnc,'-g');
title('blue-des,green-enc');