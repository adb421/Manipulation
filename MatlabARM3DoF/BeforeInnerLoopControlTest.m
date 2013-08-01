%%
%This script will run the arm through a basic sinusoidal control in "x"
%position
%This will test how good the control is first, and we will use the results to
%compare how well our desired x, y, and theta accelerations match the actual
%robot
%Then we can decide if an "inner loop" controller needs to be added
clear classes
%set up times for the trajectory
dt = 0.001;
params = ParametersFunction;
T = 5;
t = dt:dt:T;

%Create pc104 object and connect
pc104 = PC104_Arm3DoF;
pc104.connect();
pc104.resetEncoders();
pc104.sendControlGains();

%Create trajectory
freq = 3;
yTraj = -0.1*ones(size(t));
thTraj = zeros(size(t));
xTraj = -0.1*sin(2*pi*t*freq) - 0.25;
% xTraj = -0.25*ones(size(t));
num_pts = length(t);

home = [xTraj(1) yTraj(1) thTraj(1)];
pc104.sendHomePos(home);

disp('unpause to go home')
pause
pc104.goHome();

%Send Trajectory
pc104.allocateTraj(num_pts);
pc104.sendTraj(xTraj,yTraj,thTraj,t);

disp('unpause for traj go')
pause
%pause(15)
%Execute!
pc104.goTraj();
pause(T);
pc104.getTrajData();
pc104.killProgram();
pc104.plotTrajData();
%%

% %put this in pc104 plot
% %Calculate accelerations
% xTrajAccel = derivative(derivative(xTraj))/dt/dt;
% yTrajAccel = derivative(derivative(yTraj))/dt/dt;
% thTrajAccel = derivative(derivative(thTraj))/dt/dt;
% xControlAccel = pc104.xAccControl;
% yControlAccel = pc104.yAccControl;
% thControlAccel = pc104.thAccControl;
% th1ControlAccel = pc104.accControl1;
% th2ControlAccel = pc104.accControl2;
% th3ControlAccel = pc104.accControl3;
% xActual = -pc104.params.L1*cos(pc104.pos1) - pc104.params.L2*cos(pc104.pos1 ...
%     + pc104.pos2);
% yActual = -pc104.params.L1*sin(pc104.pos1) - pc104.params.L2*sin(pc104.pos1 ...
%     + pc104.pos2);
% thActual = pc104.pos1+pc104.pos2+pc104.pos3;
% xActualAccel = derivative(derivative(xActual))/dt/dt;
% yActualAccel = derivative(derivative(yActual))/dt/dt;
% thActualAccel = derivative(derivative(thActual))/dt/dt;
% th1ActualAccel = derivative(derivative(pc104.pos1))/dt/dt;
% th2ActualAccel = derivative(derivative(pc104.pos2))/dt/dt;
% th3ActualAccel = derivative(derivative(pc104.pos3))/dt/dt;
% 
% % %Filter that ish
% % filterNum = 30;
% % b = ones(filterNum,1)/filterNum;
% % xFiltAccel = filter(b,1,xActualAccel);
% % yFiltAccel = filter(b,1,yActualAccel);
% % thFiltAccel = filter(b,1,thActualAccel);
% % th1FiltAccel = filter(b,1,th1ActualAccel);
% % th2FiltAccel = filter(b,1,th2ActualAccel);
% % th3FiltAccel = filter(b,1,th3ActualAccel);
% 
% %Filter using FFT and FFT inverse
% accelFFTx = fft(xActualAccel);
% accelFFTy = fft(yActualAccel);
% accelFFTth = fft(thActualAccel);
% accelFFTth1 = fft(th1ActualAccel);
% accelFFTth2 = fft(th2ActualAccel);
% accelFFTth3 = fft(th3ActualAccel);
% cutoff = 40;
% index_cutoff = cutoff*T;
% accelFFTx(index_cutoff:end-index_cutoff) = ...
%     complex(zeros(size(accelFFTx(index_cutoff:end-index_cutoff))));
% accelFFTy(index_cutoff:end-index_cutoff) = ...
%     complex(zeros(size(accelFFTy(index_cutoff:end-index_cutoff))));
% accelFFTth(index_cutoff:end-index_cutoff) = ...
%     complex(zeros(size(accelFFTth(index_cutoff:end-index_cutoff))));
% accelFFTth1(index_cutoff:end-index_cutoff) = ...
%     complex(zeros(size(accelFFTth1(index_cutoff:end-index_cutoff))));
% accelFFTth2(index_cutoff:end-index_cutoff) = ...
%     complex(zeros(size(accelFFTth2(index_cutoff:end-index_cutoff))));
% accelFFTth3(index_cutoff:end-index_cutoff) = ...
%     complex(zeros(size(accelFFTth3(index_cutoff:end-index_cutoff))));
% xFiltAccel = ifft(accelFFTx, 'symmetric');
% yFiltAccel = ifft(accelFFTy, 'symmetric');
% thFiltAccel = ifft(accelFFTth, 'symmetric');
% th1FiltAccel = ifft(accelFFTth1, 'symmetric');
% th2FiltAccel = ifft(accelFFTth2, 'symmetric');
% th3FiltAccel = ifft(accelFFTth3, 'symmetric');
% 
% figure
% subplot(3,1,1)
% plot(t,xFiltAccel,'-r',t,xControlAccel,'-b');
% title('x Accelerations'); xlabel('Time (s)'); ylabel('m/s^2');
% legend('Filtered','Control');
% subplot(3,1,2)
% plot(t,yFiltAccel,'-r',t,yControlAccel,'-b');
% title('y Accelerations'); xlabel('Time (s)'); ylabel('m/s^2');
% legend('Filtered','Control');
% subplot(3,1,3)
% plot(t,thFiltAccel,'-r',t,thControlAccel,'-b');
% title('th Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
% legend('Filtered','Control');
% 
% figure
% subplot(3,1,1)
% plot(t,th1FiltAccel,'-r',t,th1ControlAccel,'-b');
% title('th1 Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
% legend('Filtered','Control');
% subplot(3,1,2)
% plot(t,th2FiltAccel,'-r',t,th2ControlAccel,'-b');
% title('th2 Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
% legend('Filtered','Control');
% subplot(3,1,3)
% plot(t,th3FiltAccel,'-r',t,th3ControlAccel,'-b');
% title('th3 Accelerations'); xlabel('Time (s)'); ylabel('rad/s^2');
% legend('Filtered','Control');