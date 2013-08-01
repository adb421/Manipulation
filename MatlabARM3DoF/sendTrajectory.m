% Given a trajectory in the plane in cartesian coordinates and an angular
% trajectory for the last link (in radians), a time vector, timestep, and
% 'ELBOW_UP' or 'ELBOW_DOWN' converts the trajectory into joint space and
% delivers it to the 3DoF arm, receives and outputs actual joint positions

function [pos1, pos2, pos3, theta1, theta2, theta3, ...
    control1, control2, control3] = ...
    sendTrajectory(x,y,theta,t,armConfig, kp, kd, ki, params)
    
    pause on
    
    %Generate the trajectory in configuration space given spatial
    %trajectory
%     [theta1 theta2 theta3] = generateTrajectory(x,y,theta,armConfig, params);
    theta1 = x;
    theta2 = y;
    theta3 = theta;
    
    %If its open already, close it, so we can reopen it
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    
    COM = serial('COM12'); %com for the 3DoF arm
    set(COM,'BaudRate',115200); %Same as setting on PIC
    set(COM,'OutputBufferSize',100000);
    fopen(COM);
    
    %Get number of points
    NUM_PTS = length(t);
    
    %In order to send control gains, send a 6, the pic will write "CONTROL"
    %Then write gains in this order:
    %kp1, kp2, kp3, kd1, kd2, kd3, ki1, ki2, ki3
    %Note these all (maybe not so much anymore) need to be singles
    %The pic will then write back "GAINSUPDATED"
    cmd = 6;
    fprintf(COM,'%d\r',cmd);
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'CONTROL')))
        fprintf(COM,'%d\r',cmd);
        text = fscanf(COM,'%s');
    end
    for i = 1:3
        fprintf(COM,'%f\r',kp(i));
    end
    for i = 1:3
        fprintf(COM,'%f\r',kd(i));
    end
    for i = 1:3
        fprintf(COM,'%f\r',ki(i));
    end
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'GAINSUPDATED')))
        fprintf(COM,'%d\r',cmd);
        text = fscanf(COM,'%s');
    end
    disp('Controls updated');
    
    %Tell the pic we are going to send it updated home positions by sending
    %a 7. Then we will tell it to go home. Our home position will be the
    %start of the trajectory.
    cmd = 7;
    fprintf(COM,'%d\r',cmd);
    fprintf(COM,'%f\r',0.0);
    fprintf(COM,'%f\r',0.0);
    fprintf(COM,'%f\r',0.0);
%     fprintf(COM,'%f\r',theta1(1));
%     fprintf(COM,'%f\r',theta2(1));
%     fprintf(COM,'%f\r',theta3(1));
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'HOMEUPDATED')))
        text = fscanf(COM,'%s');
    end
    disp('Home positions updated');
    
    %Tell the PIC to go home by sending a 4, however, right now it is only 
    %doing 0 control because we don't want it to move anywhere when this is
    %happening. Both LEDs should be off. The pic will then write back:
    %"HOME"
%     fwrite(COM,4,'int32'); %Old mode
    %cmd = 4;
    %fprintf(COM,'%d\r',cmd);
    %Check for the PIC writing back what we want
    %text = fscanf(COM,'%s');
    %while(isempty(strfind(text,'HOME')))
        %fprintf(COM,'%d\r',cmd);
        %text = fscanf(COM,'%s');
    %end
    %disp('Going home');
%     disp('No current');
%     pause
    %Not sure if pauses are necessary, but they don't hurt
    pause(0.1);
    %Tell pic you want to send the number of data points by sending a 1 to
    %the pic. It will then wait for you to send the number of data points,
    %use the new method. It will then send back the number of points it
    %received. It will then initialize data arrays. After initializing, it
    %will send back "MEMWIN" if initializing was succesfull, and both LEDs
    %will be off, otherwise it will send back "MEMFAIL" and both leds will
    %be on.
%     fwrite(COM,1,'int32'); %Old method
    cmd = 1;
    fprintf(COM,'%d\r',cmd);
    
    %Send number of data points
    %fwrite(COM,NUM_PTS,'uint32'); %Old method
    fprintf(COM,'%d\r',NUM_PTS);
    
    %Make sure we get the right number of data points
    reading = fscanf(COM,'%i',[1 1]);
    disp(['Ready to receieve ', num2str(reading), ' data points']);
    %Make sure 
    text = fscanf(COM,'%s');
    if(~isempty(strfind(text,'MEMWIN')))
        disp('Initialization succesful')
    elseif(~isempty(strfind(text,'MEMFAIL')))
        disp('Not enough memory, initialization unsuccesful. Quitting.')
        return
    else
        disp('Error, received improper string. Quitting.');
        disp(text);
        return
    end
    
    %Send a pic 2 to tell it to receive joint trajectories, this should be
    %done after sending the number of data points. Send a 2, the LEDs will
    %turn off, the pic will send us "SENDDATA" then it will receive the
    %trajectories. After receiving each trajectory we will receive a
    %"DONETRAJ1", "DONETRAJ2", or "DONETRAJ3".
%     fwrite(COM,2,'int32');
    cmd = 2;
    fprintf(COM,'%d\r',cmd);
    text = fscanf(COM, '%s');
    while(isempty(strfind(text,'SENDDATA')))
        fprintf(COM,'%d\r',cmd);
        text = fscanf(COM,'%s');
    end
    disp('Sending data')
    
    %Send joint angles
    for i = 1:NUM_PTS
        fprintf(COM,'%f\r',theta1(i));
    end
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'DONETRAJ1')))
        text = fscanf(COM,'%s');
    end
    disp('Sent angle 1');
    for i = 1:NUM_PTS
       fprintf(COM,'%f\r',theta2(i));
    end
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'DONETRAJ2')))
        text = fscanf(COM,'%s');
    end
    disp('Sent angle 2');
    for i = 1:NUM_PTS
        fprintf(COM,'%f\r',theta3(i));
    end
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'DONETRAJ3')))
        text = fscanf(COM,'%s');
    end
    disp('Sent angle 3');
    
    pause
    
    %Check the position of the motors
    cmd = 8;
    fprintf(COM,'%d\r',cmd);
    disp(fscanf(COM,'%s'));
    disp(fscanf(COM,'%s'));
    disp(fscanf(COM,'%s'));
     
    %Send the pic a 5 to execute the trajectory. The pic will send back
    %"EXECUTE" in confirmation. Trajectory control will begin.
%     fwrite(COM,5,'int32');
    cmd = 5;
    fprintf(COM,'%d\r',cmd);
    text = fscanf(COM, '%s');
    while(isempty(strfind(text,'EXECUTE')))
%         fwrite(COM,5,'int32');
        fprintf(COM,'%d\r',cmd);
        text = fscanf(COM,'%s');
    end
    disp('Executing trjaectory');
    
    %pause for max duration and a slight extra amount of time
    pause(t(end) + 0.25);
    
    %Write a 3 for the PIC to send back the actual position data. The pic
    %will turn off any controls and turn led1 off and led2 on. It will then
    %write "START", all of position1, then position2, then position3, then
    %"END". The pic will then "reset"(not really a reset, just frees some
    %memory and handles a few other things) and send back "RESET"
%     fwrite(COM,3,'int32');
    cmd = 3;
    fprintf(COM,'%d\r',cmd);
    
    %wait until we read start command
    text = fscanf(COM, '%s');
    while(isempty(strfind(text,'START')))
        text = fscanf(COM, '%s');
    end
    
    disp('Start receiving data');
    %initialize data vector as empty
    pos1 = zeros(1,NUM_PTS);
    pos2 = zeros(1,NUM_PTS);
    pos3 = zeros(1,NUM_PTS);
    control1 = zeros(1,NUM_PTS);
    control2 = zeros(1,NUM_PTS);
    control3 = zeros(1,NUM_PTS);
    for i = 1:NUM_PTS
        reading = fscanf(COM, '%f',[1 1]);
        pos1(i) = reading;
    end
    for i = 1:NUM_PTS
        reading = fscanf(COM, '%f',[1 1]);
        pos2(i) = reading;
    end
    for i = 1:NUM_PTS
        reading = fscanf(COM, '%f',[1 1]);
        try
            pos3(i) = reading;
        catch err
            disp(size(reading));
            rethrow(err);
        end
    end
    last = fscanf(COM,'%s');
    if strcmp(last,'POSEND')
        disp('Got positions')
    else
        disp('Error');
    end
    for i = 1:NUM_PTS
        reading = fscanf(COM, '%d',[1 1]);
        control1(i) = reading;
    end
    for i = 1:NUM_PTS
        reading = fscanf(COM, '%d');
        try
            control2(i) = reading;
        catch err
            disp(size(reading));
            rethrow(err);
        end
    end
    for i = 1:NUM_PTS
        reading = fscanf(COM, '%d',[1 1]);
        try
            control3(i) = reading;
        catch err
            readError = reading;
            rethrow(err);
        end
    end
    last = fscanf(COM,'%s');
    if strcmp(last,'END')
        disp('Got data')
    else
        disp('Issue receiving data')
    end
    
    
    text = fscanf(COM,'%s');
    while(isempty(strfind(text,'RESET')))
        disp(text)
        text = fscanf(COM,'%s');
    end
    disp('RESET');
    
    pause(0.1)
    
    %Tell the PIC to go home by sending a 4, however, right now it is only 
    %doing 0 control because we don't want it to move anywhere when this is
    %happening. Both LEDs should be off. The pic will then write back:
    %"HOME"
%     fwrite(COM,4,'int32'); %Old mode
%     cmd = 4;
%     fprintf(COM,'%d\r',cmd);
%     %Check for the PIC writing back what we want
%     text = fscanf(COM,'%s');
%     while(isempty(strfind(text,'HOME')))
%         fprintf(COM,'%d\r',cmd);
%         text = fscanf(COM,'%s');
%     end
%     disp('Going home');
    
    
    fclose(COM);
end