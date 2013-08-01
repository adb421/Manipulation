% Script to play the video, and plot what's going on!
load('CompareToMovie_One_Point_Balance_6_20_2013.mat');
videoFile = ...
    VideoReader('C:\Users\Adam\Desktop\OnePointRollVideos\OnePointBalance_6_20_2013_edited.avi');
video = read(videoFile);
fixCameraData;
params = pc104.params;

%Make a subplot, top left corner is movie frame
%Bottom right is animation
%Top Middle is x/y pos of object
%Bottom Middle is manipulator x/y desired accelerations
%Top right is theta pos of object
%Bottom right is manipulator theta desired accelerations
screen_size = get(0,'ScreenSize');
fig = figure;
set(fig,'Position',[0 0 screen_size(3)*.75 screen_size(4)*.75]);
moviePlot = subplot(2,3,1);
vidFrame = image(video(:,:,:,1));
animPlot = subplot(2,3,4);
xm = -params.L1*cos(pc104.pos1(indices)) - ...
    params.L2*cos(pc104.pos1(indices) + pc104.pos2(indices));
ym = -params.L1*sin(pc104.pos1(indices)) - ...
    params.L2*sin(pc104.pos1(indices) + pc104.pos2(indices));
thm = pc104.pos3(indices) + pc104.pos2(indices) + pc104.pos1(indices);
xo = fixedObjX;
yo = fixedObjY;
tho = fixedObjTh;
axis off, axis equal
goalAngle = pi/2-atan(params.wo/params.lo);
xverts = [-0.25 - params.lm; -0.25 + params.lm; ...
          -0.25 + params.lm; -0.25 - params.lm];
yDesAnim = -0.1-params.wm - params.lc;
yverts = [yDesAnim-params.wm; yDesAnim-params.wm; ...
          yDesAnim+params.wm; yDesAnim+params.wm];
manipGoal = patch(xverts,yverts,'r');
hold on

xverts = [-0.25 - params.lo*cos(goalAngle) + params.wo*sin(goalAngle);
          -0.25 + params.lo*cos(goalAngle) + params.wo*sin(goalAngle);
          -0.25 + params.lo*cos(goalAngle) - params.wo*sin(goalAngle);
          -0.25 - params.lo*cos(goalAngle) - params.wo*sin(goalAngle)];
yverts = [-0.1 - params.wo*cos(goalAngle) - params.lo*sin(goalAngle);
          -0.1 - params.wo*cos(goalAngle) + params.lo*sin(goalAngle);
          -0.1 + params.wo*cos(goalAngle) + params.lo*sin(goalAngle);
          -0.1 + params.wo*cos(goalAngle) - params.lo*sin(goalAngle)];
objGoal = patch(xverts,yverts,'b');
xverts = [xm(1) - params.lm*cos(thm(1)) + params.wm*sin(thm(1)); ...
          xm(1) + params.lm*cos(thm(1)) + params.wm*sin(thm(1)); ...
          xm(1) + params.lm*cos(thm(1)) - params.wm*sin(thm(1));
          xm(1) - params.lm*cos(thm(1)) - params.wm*sin(thm(1))];
yverts = [ym(1) - params.wm*cos(thm(1)) - params.lm*sin(thm(1)); ...
          ym(1) - params.wm*cos(thm(1)) + params.lm*sin(thm(1)); ...
          ym(1) + params.wm*cos(thm(1)) + params.lm*sin(thm(1));
          ym(1) + params.wm*cos(thm(1)) - params.lm*sin(thm(1))];
manip = patch(xverts,yverts,'r');
xverts = [xo(1) - params.lo*cos(tho(1)) + params.wo*sin(tho(1)); ...
          xo(1) + params.lo*cos(tho(1)) + params.wo*sin(tho(1)); ...
          xo(1) + params.lo*cos(tho(1)) - params.wo*sin(tho(1));
          xo(1) - params.lo*cos(tho(1)) - params.wo*sin(tho(1))];
yverts = [yo(1) - params.wo*cos(tho(1)) - params.lo*sin(tho(1)); ...
          yo(1) - params.wo*cos(tho(1)) + params.lo*sin(tho(1)); ...
          yo(1) + params.wo*cos(tho(1)) + params.lo*sin(tho(1));
          yo(1) + params.wo*cos(tho(1)) - params.lo*sin(tho(1))];
obj = patch(xverts,yverts,'b');
title('Animation based on Encoders and Camera');
xlim([-0.5 0]); ylim([-0.5 0.5]);
xlabel('x position (m)'); ylabel('y position(m)');



posPlot = subplot(2,3,2);
plot(tcam,fixedObjX,'r-',tcam,fixedObjY,'b-', ...
    tcam, -0.25*ones(size(tcam)),'r--',tcam,-0.1*ones(size(tcam)),'b--');
legend('X Position','Y Position');
grid on
hold on
linePos = line([tcam(1) tcam(1)],get(posPlot,'YLim'), 'Color','k');
hold off
title('Position of Object from Camera');
xlabel('Time (s)'); ylabel('Position (m)');

controlPlot = subplot(2,3,5);
plot(pc104.t(indices),pc104.xAccControl(indices),'-r', ...
    pc104.t(indices),pc104.yAccControl(indices),'-b');
legend('xdd','ydd');
grid on
ylim([-4 4])
hold on
lineControl = line([tcam(1) tcam(1)],get(controlPlot,'YLim'), 'Color','k');
hold off
title('Desired control accelerations');
xlabel('Time (s)'); ylabel('Accelerations (m/s^2)');

thetaPlot = subplot(2,3,3);
plot(tcam, fixedObjTh,'r-',tcam,goalAngle*ones(size(tcam)),'r--');
ylim([0.5 1.5])
grid on
hold on
lineTheta = line([tcam(1) tcam(1)],get(thetaPlot,'YLim'),'Color','k');
hold off
title('Orientation of Object from Camera');
xlabel('Time (s)'); ylabel('Orientation (rad)');

thControlPlot = subplot(2,3,6);
plot(pc104.t(indices),pc104.thAccControl(indices),'-r');
grid on
hold on
lineThControl = line([tcam(1) tcam(1)],get(thControlPlot,'YLim'), 'Color','k');
hold off
title('Desired angular accelerations');
xlabel('Time (s)'); ylabel('Angular Accelerations (rad/s^2)');

drawnow;
%Loop
indSkip = 10; %Want to update at 25Hz, index updates at 250Hz
movieIndex = 1;

myCount = 1;
for i = 25:indSkip:677
    j = indices(i);
    movieIndex = movieIndex + 24; 
    %Update lines
    set(linePos,'XData',[tcam(i) tcam(i)]);
    set(lineControl,'XData',[tcam(i) tcam(i)]);
    set(lineTheta,'XData',[tcam(i) tcam(i)]);
    set(lineThControl,'XData',[tcam(i) tcam(i)]);
    %Update animation
    xverts = [xm(i) - params.lm*cos(thm(i)) + params.wm*sin(thm(i)); ...
              xm(i) + params.lm*cos(thm(i)) + params.wm*sin(thm(i)); ...
              xm(i) + params.lm*cos(thm(i)) - params.wm*sin(thm(i));
              xm(i) - params.lm*cos(thm(i)) - params.wm*sin(thm(i))];
    yverts = [ym(i) - params.wm*cos(thm(i)) - params.lm*sin(thm(i)); ...
              ym(i) - params.wm*cos(thm(i)) + params.lm*sin(thm(i)); ...
              ym(i) + params.wm*cos(thm(i)) + params.lm*sin(thm(i));
              ym(i) + params.wm*cos(thm(i)) - params.lm*sin(thm(i))];
    set(manip,'XData',xverts,'YData',yverts);
    xverts = [xo(i) - params.lo*cos(tho(i)) + params.wo*sin(tho(i)); ...
              xo(i) + params.lo*cos(tho(i)) + params.wo*sin(tho(i)); ...
              xo(i) + params.lo*cos(tho(i)) - params.wo*sin(tho(i));
              xo(i) - params.lo*cos(tho(i)) - params.wo*sin(tho(i))];
    yverts = [yo(i) - params.wo*cos(tho(i)) - params.lo*sin(tho(i)); ...
              yo(i) - params.wo*cos(tho(i)) + params.lo*sin(tho(i)); ...
              yo(i) + params.wo*cos(tho(i)) + params.lo*sin(tho(i));
              yo(i) + params.wo*cos(tho(i)) - params.lo*sin(tho(i))];
    set(obj,'XData',xverts,'YData',yverts);
    %Update movie
    set(vidFrame,'CData',video(:,:,:,movieIndex));
    drawnow;
    M(myCount) = getframe(fig);
    myCount = myCount +1;
    pause(0.1)
end