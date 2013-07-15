classdef PC104_Arm3DoF < handle
    %PC104_Arm3DoF Class embodying connection to PC104
    %   Includes all methods and variables needed to do communication with
    %   the PC104 stack which is handling low level 3 Degree of Freedom arm
    %   control
    
    properties
        port_in;
        port_out;
        dt;
        IPin;
        IPout;
        params = ParametersFunction();
        connected;
        num_pts;
        traj1; traj2; traj3;
        checkTraj1; checkTraj2; checkTraj3;
        pos1; pos2; pos3;
        controlVals1; controlVals2; controlVals3;
        camPosX; camPosY; camPosTh;
        loopTimes;
        objPosX; objPosY; objPosTh;
        t;
        xAccControl, yAccControl, thAccControl;
        accControl1, accControl2, accControl3;
        k1RH14; k2RH14; k1RH11; k2RH11; k1RH8; k2RH8;
        tcam;
        fixedCamX; fixedCamY;
        fixedObjX; fixedObjY; fixedObjTh;
    end
    
    methods
        %Constructor
        function obj = PC104_Arm3DoF()
            obj.port_in = 3100;
            obj.port_out = 3490;
            obj.dt = 0.001;
            obj.connected = 0;
            obj.num_pts = 1;
        end
        %Connect to the PC104
        function connect(obj)
            obj.IPin = tcpip('192.168.1.50',obj.port_in);
            obj.IPout = tcpip('192.168.1.50',obj.port_out);
            set(obj.IPin,'ByteOrder','littleEndian');
            set(obj.IPout,'ByteOrder','littleEndian');
            set(obj.IPin,'InputBufferSize',4095);
            set(obj.IPout,'OutputBufferSize',4095);
            fopen(obj.IPin);
            fopen(obj.IPout);
            disp('Connected');
            obj.connected = 1;
        end
        
        %Send standard control gains (cmd = 6)
        function sendControlGains(obj, gains)
            if(nargin > 1)
                kp = gains(1:3); kd = gains(4:6); ki = gains(7:9);
            else
                kp = obj.params.kp; kd = obj.params.kd; ki = obj.params.ki;
            end
            cmd = 6;
            if(obj.connected)
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'CONTROL')))
                    disp('Problem updating gains');
                    disp(text);
                    return;
                end
                disp('Sending control gains')
                for i = 1:3
                    fwrite(obj.IPout,kp(i),'double');
                end
                for i = 1:3
                    fwrite(obj.IPout,kd(i),'double');
                end
                for i = 1:3
                    fwrite(obj.IPout,ki(i),'double');
                end
                %Go K1, K2, K1, K2, K1, K2 from RH14->11->8
                fwrite(obj.IPout,obj.params.k1RH14,'double');
                fwrite(obj.IPout,obj.params.k2RH14,'double');
                fwrite(obj.IPout,obj.params.k1RH11,'double');
                fwrite(obj.IPout,obj.params.k2RH11,'double');
                fwrite(obj.IPout,obj.params.k1RH8,'double');
                fwrite(obj.IPout,obj.params.k2RH8,'double');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'GAINSUPDATED')))
                    disp('Gains not updated');
                    return;
                end
            else
                disp('not connected');
                return;
            end
        end
        
        %Reset encoder values based on camera (cmd = 10)
        function resetEncoders(obj)
            if(obj.connected)
                cmd = 10;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'ENCRESET')))
                    disp('Encoders not reset');
                    disp(text)
                    return;
                end
                disp('Encoder counts reset');
            else
                disp('Not connected');
                return;
            end
        end
        
        %"Emergency" stop and reset (cmd = 9)
        function stopAndReset(obj)
            if(obj.connected)
                cmd = 9;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                while(isempty(strfind(text,'STOP')))
                    fwrite(obj.IPout,'int32');
                    text = fscanf(obj.IPin,'%s');
                end
                disp('Stopped')
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'RESET')))
                    disp('Not reset?')
                    disp(text);
                    return;
                end
                disp('Reset')
            else
                disp('Not connected')
                return
            end
        end
        
        %Send home positions (cmd = 7)
        function sendHomePos(obj,home)
            if(obj.connected)
                if(nargin == 2 && length(home) == 3)
                    cmd = 7;
                    fwrite(obj.IPout,cmd,'int32');
                    fwrite(obj.IPout,home(1),'double');
                    fwrite(obj.IPout,home(2),'double');
                    fwrite(obj.IPout,home(3),'double');
                    text = fscanf(obj.IPin,'%s');
                    if(isempty(strfind(text,'HOMEUPDATED')))
                        disp('Home not updated')
                        disp(text)
                        return
                    end
                    disp('Home positions sent')
                else
                    disp('Bad inputs to send home');
                    return
                end
            else
                disp('Not connected')
                return
            end
        end
        
        %Go home (cmd = 4)
        function goHome(obj)
            if(obj.connected)
                cmd = 4;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'HOME')))
                    disp('Not going home?')
                    disp(text)
                    return
                end
                disp('Going home!')
            else
                disp('Not connected')
                return
            end
        end
        
        %Send points and allocate (cmd = 1)
        function allocateTraj(obj,numPts)
            if(obj.connected)
                if(nargin == 2)
                    obj.num_pts = numPts;
                end
                cmd = 1;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'ALLOCATE')))
                    disp('Did not get command?')
                    disp(text)
                    return
                end
                fwrite(obj.IPout,obj.num_pts,'int32');
                text = fscanf(obj.IPin,'%s');
                if(~isempty(strfind(text,'MEMFAIL')))
                    disp('Memory not allocated');
                    disp(text)
                    return;
                end
                disp('Memory allocated');
            else
                disp('Not connected')
                return
            end
        end
        
        %Send trajectories (cmd = 2)
        function sendTraj(obj,trajIn1,trajIn2,trajIn3,t)
            if(obj.connected)
                if(nargin == 5)
                    obj.traj1 = trajIn1;
                    obj.traj2 = trajIn2;
                    obj.traj3 = trajIn3;
                    obj.t = t;
                    if(obj.num_pts ~= length(trajIn1))
                        disp('Num points not equal')
                        return
                    end
                    cmd = 2;
                    fwrite(obj.IPout,cmd,'int32');
                    text = fscanf(obj.IPin,'%s');
                    if(isempty(strfind(text,'SENDDATA')))
                        disp('Did not get command')
                        disp(text)
                        return
                    end
                    disp('Sending trajectory');
                    for i = 1:obj.num_pts
                        if(mod(i,100) == 0)
                            pause(0.1)
                        end
                        fwrite(obj.IPout,obj.traj1(i),'double');
                    end
                    text = fscanf(obj.IPin,'%s');
                    if(isempty(strfind(text,'DONETRAJ1')))
                        disp('Did not send traj1');
                        disp(text)
                        return
                    end
                    disp('Sent first trajectory');
                    for i = 1:obj.num_pts
                        if(mod(i,100) == 0)
                            pause(0.1)
                        end
                        fwrite(obj.IPout,obj.traj2(i),'double');
                    end
                    text = fscanf(obj.IPin,'%s');
                    if(isempty(strfind(text,'DONETRAJ2')))
                        disp('Did not send traj2');
                        disp(text)
                        return
                    end
                    disp('Sent second trajectory');
                    for i = 1:obj.num_pts
                        if(mod(i,100) == 0)
                            pause(0.1)
                        end
                        fwrite(obj.IPout,obj.traj3(i),'double');
                    end
                    text = fscanf(obj.IPin,'%s');
                    if(isempty(strfind(text,'DONETRAJ')))
                        disp('Did not get all traj?')
                        disp(text)
                        return
                    end
                    disp('Sent trajectories');
                else
                    disp('Bad input')
                    return
                end
            else
                disp('Not connected')
                return
            end
        end
        
        %Execute trajectory (cmd = 5)
        function goTraj(obj)
            if(obj.connected)
%                 if(isempty(obj.traj1))
                if(0)
                    disp('Havent sent trajectories');
                else
                    cmd = 5;
                    fwrite(obj.IPout,cmd,'int32');
                    text = fscanf(obj.IPin,'%s');
                    if(isempty(strfind(text,'EXECUTE')))
                        disp('Didnt get command');
                        return
                    end
                    disp(['Executing, please wait ' ...
                        num2str(obj.num_pts*obj.dt) ' s.']);
                end
            else
                disp('Not connected')
                return
            end
        end
        
        %Get data from a trajectory run (cmd = 3)
        function getTrajData(obj)
            if(obj.connected)
                cmd = 3;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'START')))
                    disp('Did not get command?')
                    disp(text)
                    return
                end
                disp('Receiving data');
                for i = 1:obj.num_pts
                    obj.pos1(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.pos2(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.pos3(i) = fread(obj.IPin,1,'double');
                end
                disp('Got encoder vals')
                for i = 1:obj.num_pts
                    obj.controlVals1(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.controlVals2(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.controlVals3(i) = fread(obj.IPin,1,'double');
                end
                disp('Got control vals')
                for i = 1:obj.num_pts
                    obj.camPosX(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.camPosY(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.camPosTh(i) = fread(obj.IPin,1,'double');
                end
                disp('Got camera manipulator positions')
                for i = 1:obj.num_pts
                    obj.loopTimes(i) = fread(obj.IPin,1,'double');
                end  
                for i = 1:obj.num_pts
                    obj.objPosX(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.objPosY(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.objPosTh(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.xAccControl(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.yAccControl(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.thAccControl(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.accControl1(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.accControl2(i) = fread(obj.IPin,1,'double');
                end
                for i = 1:obj.num_pts
                    obj.accControl3(i) = fread(obj.IPin,1,'double');
                end
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'END')))
                    disp('Did not get all data?')
                    disp(text)
                    return;
                end
                disp('Got all data, resetting');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'RESET')))
                    disp('No reset?');
                    disp(text)
                    return
                end
                disp('Reset.')
            else
                disp('Not connected')
                return
            end
        end
        
        %Get current encoder positions (cmd = 8)
        function currentEncoderPos(obj)
            if(obj.connected)
                cmd = 8;
                fwrite(obj.IPout,cmd,'int32');
                encPos1 = fread(obj.IPin,1,'double');
                encPos2 = fread(obj.IPin,1,'double');
                encPos3 = fread(obj.IPin,1,'double');
                disp(['RH14: ' num2str(encPos1)]);
                disp(['RH11: ' num2str(encPos2)]);
                disp(['RH8: ' num2str(encPos3)]);
            else
                disp('Not connected')
                return
            end
        end
    
        %Miscellaneous functions (cmd = 11), not doing anything now
        function misc(obj)
            disp('No misc function set');
        end
        
        %Send LQR gains for balancing at a point (cmd = 13)
        function LQRGainSend(obj)
            if(obj.connected) 
                A = obj.params.A;
                B = obj.params.B;
                R = obj.params.R;
                Q = obj.params.Q;
                K = lqr(A,B,Q,R);
                cmd = 13;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'LQRGAINS')))
                    disp('No LQRGAINS?')
                    disp(text)
                    return
                end
                for i = 1:3
                    for j = 1:8
                        fwrite(obj.IPout,K(i,j),'double');
                    end
                end
                %Go K1, K2, K1, K2, K1, K2 from RH14->11->8
                fwrite(obj.IPout,obj.params.k1RH14,'double');
                fwrite(obj.IPout,obj.params.k2RH14,'double');
                fwrite(obj.IPout,obj.params.k1RH11,'double');
                fwrite(obj.IPout,obj.params.k2RH11,'double');
                fwrite(obj.IPout,obj.params.k1RH8,'double');
                fwrite(obj.IPout,obj.params.k2RH8,'double');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'LQRUPDATED')))
                    disp('Didnt get LQR gains')
                    disp(text)
                    return
                end
            else
                disp('Not connected')
                return
            end
        end
        
        %New LQR gain send for balancing at a point (cmd = 13)
        function newLQRGainSend(obj)
            if(obj.connected)
                A = obj.params.Anew;
                B = obj.params.Bnew;
                R = obj.params.Rnew;
                Q = obj.params.Qnew;
                K = lqr(A,B,Q,R);
                cmd = 13;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'LQRGAINS')))
                    disp('No LQRGAINS?')
                    disp(text)
                    return
                end
                for i = 1:3
                    for j = 1:8
                        fwrite(obj.IPout,K(i,j),'double');
                    end
                end
                %K1, K2, K1, K2, K1, K2 from RH14->11->8
                fwrite(obj.IPout,obj.params.k1RH14,'double');
                fwrite(obj.IPout,obj.params.k2RH14,'double');
                fwrite(obj.IPout,obj.params.k1RH11,'double');
                fwrite(obj.IPout,obj.params.k2RH11,'double');
                fwrite(obj.IPout,obj.params.k1RH8,'double');
                fwrite(obj.IPout,obj.params.k2RH8,'double');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'LQRUPDATED')))
                    disp('Didnt get LQR gains')
                    disp(text)
                    return
                end
            else
                disp('Not connected')
                return
            end
        end
        
        
        %Plotting function
        function plotTrajData(obj)
            %Fix camera data
            obj.tcam = zeros(size(obj.t));
            obj.fixedCamX = zeros(size(obj.camPosX));
            obj.fixedCamY = zeros(size(obj.camPosY));
            obj.fixedObjX = zeros(size(obj.objPosX));
            obj.fixedObjY = zeros(size(obj.objPosY));
            obj.fixedObjTh = zeros(size(obj.objPosTh));
            obj.tcam(1) = obj.t(1);
            obj.fixedCamX(1) = obj.camPosX(1);
            obj.fixedCamY(1) = obj.camPosY(1);
            obj.fixedObjX(1) = obj.objPosX(1);
            obj.fixedObjY(1) = obj.objPosY(1);
            obj.fixedObjTh(1) = obj.objPosTh(1);
            j = 2;
            for i = 2:length(obj.camPosX)
                if(obj.camPosX(i) ~= obj.camPosX(i-1) || ...
                   obj.camPosY(i) ~= obj.camPosY(i-1) || ...
                   obj.objPosX(i) ~= obj.objPosX(i-1) || ...
                   obj.objPosY(i) ~= obj.objPosY(i-1) || ...
                   obj.objPosTh(i) ~= obj.objPosTh(i-1))
                    obj.fixedCamX(j) = obj.camPosX(i);
                    obj.fixedCamY(j) = obj.camPosY(i);
                    obj.tcam(j) = obj.t(i);
                    obj.fixedObjX(j) = obj.objPosX(i);
                    obj.fixedObjY(j) = obj.objPosY(i);
                    obj.fixedObjTh(j) = obj.objPosTh(i);
                    j = j+1;
                end
            end
            obj.fixedCamX = obj.fixedCamX(1:j-1);
            obj.fixedCamY = obj.fixedCamY(1:j-1);
            obj.tcam = obj.tcam(1:j-1);
            obj.fixedObjX = obj.fixedObjX(1:j-1);
            obj.fixedObjY = obj.fixedObjY(1:j-1);
            obj.fixedObjTh = obj.fixedObjTh(1:j-1);
            objXVel = derivative(obj.fixedObjX)./derivative(obj.tcam);
            objYVel = derivative(obj.fixedObjY)./derivative(obj.tcam);
            objThVel = derivative(obj.fixedObjTh)./derivative(obj.tcam);
            objXAccel = derivative(objXVel)./derivative(obj.tcam);
            objYAccel = derivative(objYVel)./derivative(obj.tcam);
            objThAccel = derivative(objThVel)./derivative(obj.tcam);
            % Plot joint angles first
            figure(1);
            subplot(3,1,1);
            plot(obj.t,obj.pos1,'r',obj.t,obj.pos2,'b',obj.t,obj.pos3,'g');
            title('Joint angular positions');
            xlabel('Time (s)'); ylabel('Radians');
            legend('Joint 1','Joint 2','Joint 3');
            %Plot joint velocities
            subplot(3,1,2);
            plot(obj.t,derivative(obj.pos1)/obj.dt,'r',obj.t,derivative(obj.pos2)/obj.dt,'b',...
                 obj.t,derivative(obj.pos3)/obj.dt,'g');
            title('Joint angular velocities');
            xlabel('Time (s)'); ylabel('Radians/second');
            legend('Joint 1','Joint 2','Joint 3');
            %Plot joint accelerations
            subplot(3,1,3);
            plot(obj.t,derivative(derivative(obj.pos1))/obj.dt/obj.dt,'r',...
                 obj.t,derivative(derivative(obj.pos2))/obj.dt/obj.dt,'b',...
                 obj.t,derivative(derivative(obj.pos3))/obj.dt/obj.dt,'g');
            title('Joint angular accelerations');
            xlabel('Time(s)'); ylabel('Radians/second');
            legend('Joint 1','Joint 2','Joint 3');
            %Plot camera versus encoder values for manipulator position
            xEnc = -obj.params.L1*cos(obj.pos1) - obj.params.L2*cos(obj.pos1 ...
                                                              + obj.pos2);
            yEnc = -obj.params.L1*sin(obj.pos1) - obj.params.L2*sin(obj.pos1 ...
                                                              + obj.pos2);
            thEnc = obj.pos1+obj.pos2+obj.pos3;
            
            %Plot x/y/z pos of manipulator
            figure(2);
            subplot(3,1,1);%For x
            plot(obj.tcam,obj.fixedCamX,'r',obj.t,xEnc,'b',obj.t,obj.traj1,'g');
            title('X position of manipulator, encoders versus camera');
            xlabel('Time (s)'); ylabel('Meters from base');
            legend('Camera','Encoders','Desired');
            subplot(3,1,2);%For y
            plot(obj.tcam,obj.fixedCamY,'r',obj.t,yEnc,'b',obj.t,obj.traj2,'g');
            title('Y position of manipulator, encoders versus camera');
            xlabel('Time (s)'); ylabel('Meters from base');
            legend('Camera','Encoders','Desired');
            subplot(3,1,3);%For orientation
            plot(obj.t,thEnc,'b',obj.t,obj.traj3,'g');
            title('Angular position of manipulator, encoders versus camera');
            xlabel('Time (s)'); ylabel('Radians');
            legend('Encoders','Desired');
            
            %Control values and looptimes
            figure(3);
            subplot(2,1,1);
            plot(obj.t,obj.controlVals1,'r',obj.t,obj.controlVals2,'b',obj.t, ...
                 obj.controlVals3,'g');
            title('Control values');
            xlabel('Time (s)'); ylabel('Amps');
            legend('Joint 1','Joint 2','Joint 3');
            subplot(2,1,2);
            plot(obj.t,obj.loopTimes);
            title('Loop times');
            xlabel('Time (s)'); ylabel('Time to complete loop (ms)');
            
            %Object location
            figure(4);
            subplot(3,1,1);
            %x Position
%             plot(obj.t,obj.traj1,'r',obj.t,obj.objPosX,'--b');
            plot(obj.t,-0.2*ones(size(obj.t)),'r', ...
                obj.tcam,obj.fixedObjX,'--b');
            title('X Position of object CoM');
            xlabel('Time (s)'); ylabel('Meters');
            legend('Desired','Camera');
            %y Position
            subplot(3,1,2);
%             plot(obj.t,obj.traj2,'r',obj.t,obj.objPosY,'--b');
            plot(obj.t,-0.1*ones(size(obj.t)),'r', ...
                obj.tcam,obj.fixedObjY,'--b');
            title('Y Position of object CoM');
            xlabel('Time (s)'); ylabel('Meters');
            legend('Desired','Camera');
            %Orientation
            subplot(3,1,3);
%             plot(obj.t,obj.traj3,'r',obj.t,obj.objPosTh,'--b');
            plot(obj.t,1.0218*ones(size(obj.t)),'r', ...
                obj.tcam,obj.fixedObjTh,'--b');
            title('Orientation of object');
            xlabel('Time (s)'); ylabel('Radians');
            legend('Desired','Camera');
            
            %Object velocity
            figure(5);
            subplot(3,1,1);
            %x Position
%             plot(obj.t,derivative(obj.traj1)/obj.dt,'r',obj.t, ...
%                  derivative(obj.objPosX)/obj.dt,'--b');
            plot(obj.t,zeros(size(obj.t)),'r', ...
                obj.tcam, objXVel,'--b');
            title('X Velocity of object CoM');
            xlabel('Time (s)'); ylabel('Meters/second');
            legend('Desired','Camera');
            %y Position
            subplot(3,1,2);
%             plot(obj.t,derivative(obj.traj2)/obj.dt,'r',obj.t, ...
%                  derivative(obj.objPosY)/obj.dt,'--b');
            plot(obj.t,zeros(size(obj.t)),'r', ...
                obj.tcam, objYVel,'--b');
            title('Y Velocity of object CoM');
            xlabel('Time (s)'); ylabel('Meters/second');
            legend('Desired','Camera');
            %Orientation
            subplot(3,1,3);
%             plot(obj.t,derivative(obj.traj3)/obj.dt,'r',obj.t, ...
%                  derivative(obj.objPosTh)/obj.dt,'--b');
            plot(obj.t,zeros(size(obj.t)),'r', ...
                obj.tcam, objThVel,'--b');
            title('Angular velocity of object');
            xlabel('Time (s)'); ylabel('Radians/second');
            legend('Desired','Camera');
            
            %Object acceleration
            figure(6);
            subplot(3,1,1);
            %x Position
%             plot(obj.t,derivative(derivative(obj.traj1)/obj.dt)/obj.dt,'r',obj.t, ...
%                  derivative(derivative(obj.objPosX)/obj.dt)/obj.dt,'--b');
            plot(obj.t,zeros(size(obj.t)),'r', ...
                obj.tcam, objXAccel,'--b');
            title('X Acceleration of object CoM');
            xlabel('Time (s)'); ylabel('Meters/second^2');
            legend('Desired','Camera');
            %y Position
            subplot(3,1,2);
%             plot(obj.t,derivative(derivative(obj.traj2)/obj.dt)/obj.dt,'r',obj.t, ...
%                  derivative(derivative(obj.objPosY)/obj.dt)/obj.dt,'--b');
            plot(obj.t,zeros(size(obj.t)),'r', ...
                obj.tcam, objYAccel,'--b');
            title('Y Acceleration of object CoM');
            xlabel('Time (s)'); ylabel('Meters/second^2');
            legend('Desired','Camera');
            %Orientation
            subplot(3,1,3);
%             plot(obj.t,derivative(derivative(obj.traj3)/obj.dt)/obj.dt,'r',obj.t, ...
%                  derivative(derivative(obj.objPosTh)/obj.dt)/obj.dt,'--b');
            plot(obj.t,zeros(size(obj.t)),'r', ...
                obj.tcam, objThAccel,'--b');
            title('Angular accleration of object');
            xlabel('Time (s)'); ylabel('Radians/second^2');
            legend('Desired','Camera');
            
            %Object in world x-y
            figure(7);
            plot(obj.traj1,obj.traj2,'r',obj.fixedObjX,obj.fixedObjY,'--b', ...
                obj.fixedCamX,obj.fixedCamY,'--g',xEnc,yEnc,'g');
            title('Position in world frame');
            xlabel('x (m)'); ylabel('y (m)');
            legend('Desired','Camera Object','Cam manip', 'Enc manip');
            
            %How is acceleration control working?
            xTrajAccel = derivative(derivative(obj.traj1))/obj.dt/obj.dt;
            yTrajAccel = derivative(derivative(obj.traj2))/obj.dt/obj.dt;
            thTrajAccel = derivative(derivative(obj.traj3))/obj.dt/obj.dt;
            xActualAccel = derivative(derivative(xEnc))/obj.dt/obj.dt;
            yActualAccel = derivative(derivative(yEnc))/obj.dt/obj.dt;
            thActualAccel = derivative(derivative(thEnc))/obj.dt/obj.dt;
            th1ActualAccel = derivative(derivative(obj.pos1))/obj.dt/obj.dt;
            th2ActualAccel = derivative(derivative(obj.pos2))/obj.dt/obj.dt;
            th3ActualAccel = derivative(derivative(obj.pos3))/obj.dt/obj.dt;
            
            %Filter via FFTs
            accelFFTx = fft(xActualAccel);
            accelFFTy = fft(yActualAccel);
            accelFFTth = fft(thActualAccel);
            accelFFTth1 = fft(th1ActualAccel);
            accelFFTth2 = fft(th2ActualAccel);
            accelFFTth3 = fft(th3ActualAccel);
            cutoff = 40; %for now
            index_cutoff = cutoff*obj.t(end);
            %Zero out high frequency information
            accelFFTx(index_cutoff:end-index_cutoff) = ...
                complex(zeros(size(accelFFTx(index_cutoff:end-index_cutoff))));
            accelFFTy(index_cutoff:end-index_cutoff) = ...
                complex(zeros(size(accelFFTy(index_cutoff:end-index_cutoff))));
            accelFFTth(index_cutoff:end-index_cutoff) = ...
                complex(zeros(size(accelFFTth(index_cutoff:end-index_cutoff))));
            accelFFTth1(index_cutoff:end-index_cutoff) = ...
                complex(zeros(size(accelFFTth1(index_cutoff:end-index_cutoff))));
            accelFFTth2(index_cutoff:end-index_cutoff) = ...
                complex(zeros(size(accelFFTth2(index_cutoff:end-index_cutoff))));
            accelFFTth3(index_cutoff:end-index_cutoff) = ...
                complex(zeros(size(accelFFTth3(index_cutoff:end-index_cutoff))));
            xFiltAccel = ifft(accelFFTx, 'symmetric');
            yFiltAccel = ifft(accelFFTy, 'symmetric');
            thFiltAccel = ifft(accelFFTth, 'symmetric');
            th1FiltAccel = ifft(accelFFTth1, 'symmetric');
            th2FiltAccel = ifft(accelFFTth2, 'symmetric');
            th3FiltAccel = ifft(accelFFTth3, 'symmetric');
            
            %Now plot some accelerations
            figure(8)%cartesian
            subplot(3,1,1)%x
            plot(obj.t,xFiltAccel,'-r',obj.t,obj.xAccControl,'-b');
            title('x Accelerations'); xlabel('Time(s)'); ylabel('m/s^2');
            legend('Filtered','Control');
            subplot(3,1,2)%y
            plot(obj.t,yFiltAccel,'-r',obj.t,obj.yAccControl,'-b');
            title('y Accelerations'); xlabel('Time(s)'); ylabel('m/s^2');
            legend('Filtered','Control');
            subplot(3,1,3)%th
            plot(obj.t,thFiltAccel,'r',obj.t,obj.thAccControl,'-b');
            title('th Accelerations'); xlabel('Time(s)'); ylabel('rad/s^2');
            legend('Filtered','Control');
            
            figure(9)%joint
            subplot(3,1,1)%th1
            plot(obj.t,th1FiltAccel,'-r',obj.t,obj.accControl1,'-b');
            title('Joint 1 Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
            legend('Filtered','Control');
            subplot(3,1,2)%th2
            plot(obj.t,th2FiltAccel,'-r',obj.t,obj.accControl2,'-b');
            title('Joint 2 Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
            legend('Filtered','Control');
            subplot(3,1,3)%th3
            plot(obj.t,th3FiltAccel,'-r',obj.t,obj.accControl3,'-b');
            title('Joint 3 Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
            legend('Filtered','Control');
        end
        
        %Kill PC104 program
        function killProgram(obj)
            if(obj.connected)
                cmd = 14;
                fwrite(obj.IPout,cmd,'int32');
                text = fscanf(obj.IPin,'%s');
                if(isempty(strfind(text,'KILL')))
                    disp('Didnt get kill command')
                    disp(text)
                    return
                end
                disp('Program will end within 1 second')
            else
                disp('Not connected')
                return
            end
        end
    end
end

