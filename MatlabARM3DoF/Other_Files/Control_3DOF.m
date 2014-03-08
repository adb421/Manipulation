% Wei Tong's original code to communicate with the PIC32 controlled arm. WAY WAY WAY DEPRECATED
%--------------------------------------------------------------------------
%   2DOF_Final.m
%   Wei Tong
%   User chooses the desired arm config, home position, and trajectory for
%   the arm.
%   This program will calculate the required torques for the arm and will
%   then send the data through RS232.
%   The program will then get the output data from the arm to be plotted.
%--------------------------------------------------------------------------
% ARM CONSTANTS
clear;
a1 = 19.3675;  % Link legnth in cm
a2 = 19.685;    % cm
l1 = 0.193675; % Link length in m
l2 = 0.19685;

Theta1_Max = 107.57;    % Rotational constraints in degrees
Theta2_Max = 139.64;

Restx = 0;          % Rest position of arm, pointing downwards
Resty = -(a1+a2);
thetaorig1 = -pi/2;  % RH14 rest position
thetaorig2 = 0;      % RH11

mm1 = 0.5576; % Mass of motor (RH11) + bushing in kg
mm2 = 0.3945; % Mass of motor (RH8) + bushing + link 3 mass
ml1 = 0.387; % Mass of link 1, assuming the mass is evenly distributed
ml2 = 0.183; % Mass of link 2, assuming the mass is evenly distributed

g = -9.81; % <<--------------Change according to the orientation of the arm

Max_Current_RH14 = 5.4; % Amps
Max_Current_RH11 = 2.4; % Amps
km1 = 2.92; % Motor Constant, RH14, Nm/A
km2 = 2.46; % Motor Constant, RH11
RH11 = 11/1000; % Inertia at output shaft, kg·m²
RH14 = 21.6/1000;

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%   <<<<<<<<<<<<<<<    USER INPUTS     >>>>>>>>>>>>

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%--------------------------------------------------------------------------
% Choose Arm Configuration
Arm_Config  = 'ELBOW_UP';         % Uncomment to select
%Arm_Config  = 'ELBOW_DOWN';     %Uncomment to select

%--------------------------------------------------------------------------
% Enter desired home position, in meters
home_x = a1*sin(pi/3);	
home_y = -(a1/2)-a2;

%--------------------------------------------------------------------------
% Enter desired trajectory
duration = 3;   % seconds
dt = 0.001;     % time step, seconds

t = 0:dt:(duration-dt);
array_length = (duration/dt);

fx = 1;         fy = 1;                 % x and y frequencies, in Hz
phi_x = 0;      phi_y = -pi/2;           % x and y phases, in rad
A_x = 3;        A_y = 3;                % x and y magnitude, in cm
off_x = home_x;      off_y = home_y+3;            % x and y offset, in cm

% You can change this function to whatever you want it to be
for i = 1:array_length
    x(i) = A_x*sin(2*pi*fx*t(i) + phi_x) + off_x;           % x as a function of time, f(t)
    y(i) = A_y*sin(2*pi*fy*t(i) + phi_y) + off_y;           % y as a function of time, f(t)
end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%  <<<<     NO MORE INPUTS FROM THIS POINT!!!!      >>>>    
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

% Test join limits
arm_reach = realsqrt((home_x^2) + (home_y^2));
if (arm_reach > (a1+a2))
     fprintf('Arm reach cannot exceed 39.4cm!!!\n');
     return;
end

for i = 1:array_length
  r = realsqrt((x(i)^2) + (y(i)^2));
  if (r > 39.4)
      fprintf('Arm trajectory cannot exceed 39.4cm!!!\n');
      return;
  end
end

%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%       HOLD ARM       %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse Kinematics
% Get the angle of the new position
c2 = ((home_x.^2) + (home_y.^2) - (a1^2) - (a2^2)) ./ (2 * a1 * a2);
s2 = realsqrt(1 - (c2^2));
if strcmp(Arm_Config, 'ELBOW_UP')
    thetafull2 = atan2(-s2,c2);
elseif strcmp(Arm_Config, 'ELBOW_DOWN')
    thetafull2 = atan2(s2,c2);
end
r = realsqrt(home_x^2 + home_y^2);    
d = a1*c2 + a2;
if strcmp(Arm_Config, 'ELBOW_UP')
    thetafull12 = atan2(home_y,home_x) + atan2(-(realsqrt(r^2 - d^2)),d);
elseif strcmp(Arm_Config, 'ELBOW_DOWN')
    thetafull12 = atan2(home_y,home_x) + atan2((realsqrt(r^2 - d^2)),d);
end
thetafull1 = thetafull12 - thetafull2;
thetafull1out = thetafull1 + (pi/2);
thetafulldeg1 = 180/pi*(thetafull1);
thetafulldeg2 = 180/pi*(thetafull2);

% Check to make sure the angles do not exceed the physical constraints.
% Theta1 <= 107.57
% Theta2 <= 139.64

if  (abs(thetafulldeg1+90) > Theta1_Max)
  fprintf('Joint 1 cannot exceed 107.57 degrees when hold!!!\n');
  return;
elseif  (abs(thetafulldeg2) > Theta2_Max)
  fprintf('Joint 2 cannot exceed 139.64 degrees when hold!!!\n');
  return;
end

% Calculate PWM for stationary arm
FINALRH14 = 500 + (500*((mm1 + mm2 + (ml1/2) + ml2)*g*l1*cos(thetafull1)...
            + (mm2 + (ml2/2))*g*l2*cos(thetafull1+thetafull2))/km1/Max_Current_RH14);

FINALRH11 = 500 - (500*(g*l2*cos(thetafull1+thetafull2)*(mm2 + (ml2/2)))/km2/Max_Current_RH11);


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%       MOVE ARM       %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse Kinematics
for i=1:array_length
    c2 = ((x(i).^2) + (y(i).^2) - (a1^2) - (a2^2)) ./ (2 * a1 * a2);
    s2 = realsqrt(1 - (c2^2));
    if strcmp(Arm_Config, 'ELBOW_UP')
        thetarad2(i) = atan2(-s2,c2);
    elseif strcmp(Arm_Config, 'ELBOW_DOWN')
        thetarad2(i) = atan2(s2,c2);
    end
    r = realsqrt(x(i)^2 + y(i)^2);
    d = a1*cos(thetarad2(i)) + a2;
    if strcmp(Arm_Config, 'ELBOW_UP')
        theta12 = atan2(y(i),x(i)) + atan2(-(realsqrt(r^2 - d^2)),d);
    elseif strcmp(Arm_Config, 'ELBOW_DOWN')
        theta12 = atan2(y(i),x(i)) + atan2((realsqrt(r^2 - d^2)),d);
    end
    thetarad1(i) = theta12 - thetarad2(i);
    thetarad1out(i) = thetarad1(i)+(pi/2);
    % convert to degrees
    theta1(i) = 180/pi*(thetarad1(i));
    theta2(i) = 180/pi*(thetarad2(i));
    
end

% Check to make sure the angles do not exceed the physical constraints.
% Theta1 <= 107.57
% Theta2 <= 139.64
for i = 1:(array_length)
  if  (abs(theta1(i)+90) > Theta1_Max)
      fprintf('Joint 1 cannot exceed 107.57 degrees!!!\n');
      return;
  elseif  (abs(theta2(i)) > Theta2_Max)
      fprintf('Joint 2 cannot exceed 139.64 degrees!!!\n');
      return;
  end
end


%--------------------------------------------------------------------------
% Velocity and acceleration array
Vel1 = zeros(array_length,1);
Vel2 = zeros(array_length,1);
for i=2:(array_length-1)
    Vel1(i) = (thetarad1(i+1) - thetarad1(i-1))./ (2*dt);
    Vel2(i) = (thetarad2(i+1) - thetarad2(i-1))./ (2*dt); 
    if abs(Vel1(i))<= 0.0000001
        Vel1(i) = 0;
    end
    if abs(Vel2(i))<= 0.0000001
        Vel2(i) = 0;
    end
end
Vel1(1) = 0;
Vel1(array_length) = Vel1(array_length-1);
Vel2(1) = 0;
Vel2(array_length) = Vel2(array_length-1);

Acc1 = zeros(array_length,1);
Acc2 = zeros(array_length,1);
for i=2:(array_length-1)
    Acc1(i) = (thetarad1(i+1) - (2.*thetarad1(i)) + thetarad1(i-1))./(dt^2);
    Acc2(i) = (thetarad2(i+1) - (2.*thetarad2(i)) + thetarad2(i-1))./(dt^2);
    if abs(Acc1(i)) <= 0.0000001
        Acc1(i) = 0;
    end
    if abs(Acc2(i)) <= 0.0000001
        Acc2(i) = 0;
    end
end

Acc1(1) = 0;
Acc1(array_length) = Acc1(array_length-1);
Acc2(1) = 0;
Acc2(array_length) = Acc2(array_length-1);

%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%----GET required Torques++++%%%%%%%%%%%%%%%%%%%%%%%%%%
% Torque for RH14
Torque1 = zeros(array_length,1);
Torque2 = zeros(array_length,2);
for i=1:array_length
% Torque1(i) = Acc1(i)*(((mm1+mm2)*(l1^2)) + (mm2*(l2^2)) + (2*mm2*l1*l2*cos(thetarad2(i))) + ((1/3)*(ml1*(l1^2))) + (ml2*(l1^2)) + ((1/3)*(ml2*(l2^2))) + (ml2*l1*l2*cos(thetarad2(i))))...
%     + Acc2(i)*((mm2*(l2^2)) + (mm2*l1*l2*cos(thetarad2(i))) + ((1/3)*ml2*(l2^2)) + ((1/2)*ml2*l1*l2*cos(thetarad2(i))))...
%     + Vel1(i)*Vel2(i)*((-2*mm2*l1*l2*sin(thetarad2(i))) - (ml2*l1*l2*sin(thetarad2(i))))...
%     + (Vel2(i)^2)*((-mm2*l1*l2*sin(thetarad2(i))) - ((1/2)*ml2*l1*l2*sin(thetarad2(i))))...
%     + (mm1 + mm2 + (ml1/2) + ml2)*g*l1*cos(thetarad1(i))...
%     + (mm2 + (ml2/2))*g*l2*cos(thetarad1(i)+thetarad2(i))...
%     + (RH14*Acc1(i));

Torque1(i) = Acc1(i)*(RH14 + ((mm1+mm2)*(l1^2)) + (mm2*(l2^2)) + (2*mm2*l1*l2*cos(thetarad2(i))) + ((1/3)*(ml1*(l1^2))) + (ml2*(l1^2)) + ((1/3)*(ml2*(l2^2))) + (ml2*l1*l2*cos(thetarad2(i))))...
    + Acc2(i)*(RH11 + (mm2*(l2^2)) + (mm2*l1*l2*cos(thetarad2(i))) + ((1/3)*ml2*(l2^2)) + ((1/2)*ml2*l1*l2*cos(thetarad2(i))))...
    + Vel1(i)*Vel2(i)*((-2*mm2*l1*l2*sin(thetarad2(i))) - (ml2*l1*l2*sin(thetarad2(i))))...
    + (Vel2(i)^2)*((-mm2*l1*l2*sin(thetarad2(i))) - ((1/2)*ml2*l1*l2*sin(thetarad2(i))))...
    + (mm1 + mm2 + (ml1/2) + ml2)*g*l1*cos(thetarad1(i))...
    + (mm2 + (ml2/2))*g*l2*cos(thetarad1(i)+thetarad2(i));

% Torque for RH11
% Torque2(i) = Acc1(i)*((mm2*(l2^2)) + (mm2*l1*l2*cos(thetarad2(i))) + ((1/3)*ml2*(l2^2)) + ((1/2)*ml2*l1*l2*cos(thetarad2(i))))...
%     + Acc2(i)*((mm2*(l2^2)) + ((1/3)*(ml2*(l2^2))))...
%     + (Vel1(i)^2)*((mm2*l1*l2*sin(thetarad2(i))) + ((1/2)*ml2*l1*l2*sin(thetarad2(i))))...
%     + g*l2*cos(thetarad1(i)+thetarad2(i))*(mm2 + (ml2/2))...
%     + (RH11*(Acc2(i)));

Torque2(i) = Acc1(i)*(RH11 + (mm2*(l2^2)) + (mm2*l1*l2*cos(thetarad2(i))) + ((1/3)*ml2*(l2^2)) + ((1/2)*ml2*l1*l2*cos(thetarad2(i))))...
    + Acc2(i)*(RH11 + (mm2*(l2^2)) + ((1/3)*(ml2*(l2^2))))...
    + (Vel1(i)^2)*((mm2*l1*l2*sin(thetarad2(i))) + ((1/2)*ml2*l1*l2*sin(thetarad2(i))))...
    + g*l2*cos(thetarad1(i)+thetarad2(i))*(mm2 + (ml2/2));
end

%--------------------------------------------------------------------------
%!!!!!!!!!!!!!!!----Convert the torques to current----!!!!!!!!!!!!!!!!!!
Current1 = zeros(array_length,1);
Current2 = zeros(array_length,1);
PWMRH14 = zeros(array_length,1);
PWMRH11 = zeros(array_length,1);
for i=1:array_length
    % Current for RH14
    Current1(i) = Torque1(i)/km1;
    PWMRH14(i) = 500 + 500*Current1(i)/Max_Current_RH14;

% Current for RH11
    Current2(i) = Torque2(i)/km2;
    PWMRH11(i) = 500 - 500*Current2(i)/Max_Current_RH11;
    
    if ((PWMRH14(i)>1000)||(PWMRH14(i)<0))
        fprintf('Demanded current for RH14 too large!');
        return;
    end  
    if ((PWMRH11(i)>1000)||(PWMRH11(i)<0))
        fprintf('Demanded current for RH11 too large!');
        return;
    end 
end

% %--------------------------------------------------------------------------
% %--------------------------------------------------------------------------
% %--------------------------------------------------------------------------
% % <<<<<<<<<<<<!!!!!!!!!!!----- PLOT DATA ------!!!!!!!!!!!>>>>>>>>>>>>
% %-----X and Y, Theta1 and Theta2----
% figure(1);
% % subplot(2,2,[1 2]);
% hold on;
% plot(x,y);
% %axis([0 40 0 40]);
% title('Y vs X');
% xlabel('X, cm');
% ylabel('Y, cm');
% axis equal;
% 
% % subplot(2,2,3);
% % hold on;
% % plot(t,theta1);
% % title('Theta1');
% % xlabel('Time, s');
% % ylabel('Theta1, degrees');
% % 
% % subplot(2,2,4);
% % hold on;
% % plot(t,theta2);
% % title('Theta2');
% % xlabel('Time, s');
% % ylabel('Theta1, degrees');
% 
% %----Theta1 Pos, Vel, and Acc-----
% figure(2);
% subplot(3,1,1);
% plot(t,thetarad1out);
% title('Theta1 position');
% xlabel('Time, s');
% ylabel('Theta1, rad');
% 
% subplot(3,1,2);
% plot(t,Vel1);
% title('Theta1 velocity');
% xlabel('Time, s');
% ylabel('Velocity, rad/s');
% 
% subplot(3,1,3);
% plot(t,Acc1);
% title('Theta1 acceleration');
% xlabel('Time, ms');
% ylabel('Acceleration, rad/s^2');
% 
% 
% %----Theta2 Pos, Vel, and Acc-----
% figure(3);
% subplot(3,1,1);
% plot(t,thetarad2);
% title('Theta2 position');
% xlabel('Time, s');
% ylabel('Theta2, rad');
% 
% subplot(3,1,2);
% plot(t,Vel2);
% title('Theta2 velocity');
% xlabel('Time, s');
% ylabel('Velocity, rad/s');
% 
% subplot(3,1,3);
% plot(t,Acc2);
% title('Theta2 acceleration');
% xlabel('Time, s');
% ylabel('Acceleration, rad/s^2');
% 
% 
% %----Plot required Torque and current------
% figure(4);
% subplot(2,2,1);
% plot(t,Torque1);
% title('RH14 Torque');
% xlabel('Time, s');
% ylabel('Torque, Nm');
% 
% subplot(2,2,2);
% plot(t,Current1);
% title('RH14 Current');
% xlabel('Time, s');
% ylabel('Current, A');
% 
% subplot(2,2,3);
% plot(t,Torque2);
% title('RH11 Torque');
% xlabel('Time, s');
% ylabel('Torque, Nm');
% 
% subplot(2,2,4);
% plot(t,Current2);
% title('RH11 Current');
% xlabel('Time, s');
% ylabel('Current, A');
% 
% %^^^^^^^^^^^ Plot demanded PWM ^^^^^^^^^^^^^
% figure(5);
% subplot(2,1,1);
% plot(t,PWMRH14);
% title('RH14 PWM');
% xlabel('Time, s');
% ylabel('PWM');
% 
% subplot(2,1,2);
% plot(t,PWMRH11);
% title('RH11 PWM');
% xlabel('Time, s');
% ylabel('PWM');


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% RS232 Communication
% Courtesy of
% Sam Bobb, Daniel Cornew, Ryan Deeter
% Modified by Wei Tong
delete(instrfind);      % clears the old port
%This code sends and receives the required encoder position and PWM values.

columns = 1;

%check if the serial port already exists
%if it does, close it, it probably means something bad happened last run
if exist('COM','var')
    disp('closing old port instance');
    fclose(COM)
    delete(COM)
    clear COM
    disp('port closed');
end

% open the serial port
COM = serial('COM8'); % Change the COM number if necessary
set(COM,'BaudRate',115200);
set(COM,'InputBufferSize',20000);
fopen(COM);     

disp('port opened');

% send the command to run control
fprintf(COM,'p');

%there's usually some garbage at the beginning (like the echoed p 
%character command and line  breaks); this reads lines in until we find 
%the start line
text = fscanf(COM, '%s');
while isempty(strfind(text, 'START'))
    text = fscanf(COM, '%s');
end
disp('Start command received');

DATA = zeros(6000,1);

%generate the fscanf format argument
%repeate the formatchar character for the number of columns
formatchar = '%f';
serialpat = '';

%establishing fscanf format
for j = 1:columns
    serialpat = [serialpat ' ' formatchar];
end
    
%reads serial data and puts it into DATA
for j = 1:6000
%     DATA(j, :) = fscanf(COM, '%f', 4);
      %  DATA(j, :) = fread(COM, 32, 'single');
    tempData =  fscanf(COM, serialpat, [columns 1]);
    if tempData(1) == 'E'
        disp('End reached early');
        break
    end
    DATA(j, :) = tempData;
    %disp(COM.BytesAvailable)
end

%reads last line
last = fscanf(COM, '%s');

%verifies that last line read is the last line
if strcmp(last,'END')
    disp('Success')
else
    disp('Problem')
end


%closes serial port
fclose(COM);
delete(COM)
clear COM
disp('port closed');


%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Display Results

theta1rad = DATA(1:3000);       % First 3k data points are for RH14
theta2rad = DATA(3001:6000);    % Subsequent points are for RH11


% For the case of arm pointing down as the initial condition
xdata = (a1.*sin(theta1rad)) + (a2.*sin(theta1rad+theta2rad)); % kinematics
ydata = -((a1.*cos(theta1rad)) + (a2.*cos(theta1rad+theta2rad)));
figure(6);
plot(xdata,ydata,'r',x,y,'b');
title('Circular trajectory vs Ref');
xlabel('X, cm');
ylabel('Y, cm');
legend('Data','Ref');
axis equal;

figure(7);
plot(t,theta1rad,'r',t,thetarad1out,'b');
title('RH14 trajectory');
xlabel('t, ms');
ylabel('Angle, rad');
legend('Data','Ref');

figure(8);
plot(t,theta2rad,'r',t,thetarad2,'b');
title('RH11 trajectory');
xlabel('t, ms');
ylabel('Angle, rad');
legend('Data','Ref');

figure(9);
plot(t, abs(thetarad1out - theta1rad));
title('RH14 Error');
xlabel('t, ms');
ylabel('Angle Error, rad');

figure(10);
plot(t, abs(thetarad2out - theta2rad));
title('RH11 Error');
xlabel('t, ms');
ylabel('Angle Error, rad');

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% END
