************************************************************************
README FOR INTERFACING WITH PC104 ARM CONTROL
************************************************************************

The goal of the class is to abstract away all communication and low-level interfacing with the QNX PC104 stack. The class contains a great number of functions and they can sometimes be unwieldy to work through or figure out how each one works just looking at the matlab code. This will provide higher level documentation for each function and hopefully make the class easier to use. For the functions that send an int cmd to be processed by the PC104, I've included the cmd number so you can find the relevant C code relatively quickly.



Note that before using any of the functions below, it is recommended to call:
clear classes
As any changes made to the class will not be implemented if this is not called and an instance of the object exists. 


At this time I would also make sure that the Camera is connected by running the relevant c++ program.

************************************************************************
USING THE MATLAB CLASS
************************************************************************


An instance of the class can be constructed with the following command:
pc104 = PC104_Arm3DoF;

I will be using the variable name 'pc104' to refer to an object of the PC104_Arm3DoF class. You will generally only want one such object at all times.

This creates an instance of the class with the default constructor (the only constructor available). This sets the input and output ports for communication as well as the default setting of a 1kHz control loop. The object is initialized as unconnected.



One of the first things to do with the object, and probably to be done immediately after construction is the connect function:
pc104.connect();
This will attempt to connect to the QNX PC-104 over tcpip with a 1500 byte buffer for input and output, a timeout of 5 seconds, and over the ports defined in the constructor. This function takes no arguments and returns nothing. It will display 'Connected' if the connection is successful and will cause an error if not.



The PC104 code has a bunch of functions the process_commands.* files which provide ways to talk to it by sending integers. Whenever the PC104 receives a packet that is from the PC, if it is not already executing one of these functions it checks to see if it received an int that matches one of these functions. This matlab class has functions built in to send these integers over to put the PC104 in one of these functions. These are outlined below and the integers used to call them are listed with the function as "cmd = n" where n is the desired command function. These provide the easiest way to interface with the PC104 and similar functions could be added easily by following the same protocol.



pc104.allocateTraj(numPts); %cmd = 1
This function is extremely important, if must be done before any trajectory sending functions. This function tells the PC104 how large the arrays need to be so they can be allocated. numPts is the length of the trajectory, which should be the same for all the different vectors. Note that num_pts is optional if pc104.num_pts is already set.



pc104.sendTraj(trajIn1, trajIn2, trajIn3, t); %cmd = 2
Takes in 4 vectors of the same length as numPts sent to pc104.allocateTraj(numpts) (you did call this, right?). The first three are the trajectories and the last is the time vector corresponding to them. The time vector doesn't get sent over but is stored in the object locally. This is not used for modes that require 4 trajectories (like the LQR modes) but for any mode where three trajectories are needed: TRAJ_IS_CURRENT (2), FEEDFORWARD_Joints (3), PID_TRAJ_JOINTS (4), FF_PID_TRAJ_JOINTS (5), DYNAMIC_GRASP_TRAJ (6) and FF_PID_TRAJ_MANIP(8). The function will display its progress as it goes.



pc104.getTrajData(); %cmd = 3
Reads in all the data from the previous trajectory execution and stores it in the object. This will reset the program on the PC104.



pc104.goHome(); %cmd = 4
Goes to the home position as described in the pc104.sendHomePos(home) section.



pc104.goTraj(); %cmd = 5
The PC104 executes whatever trajectory was sent in whatever mode it was in. This call will not pause and should be done after the function call (pause(t(end)) for example).



pc104.sendControlGains(gains); %cmd = 6
This function sends control gains to the PC. If the variable gains is supplied, it is assumed that kp1 = gains(1), kp2 = gains(2), kp3 = gains(3), kd1 = gains(4), kd2 = gains(5), kd3 = gains(6), ki1 = gains(7), ki2 = gains(8), and ki3 = gains(9). Otherwise, the gains are pulled from the params struct which is generated in the code by calling ParametersFunction() internally. These gains are used for the following modes: GO_HOME_JOINTS (1), PID_TRAJ_JOINTS (4), FF_PID_TRAJ_JOINTS (5), DYNAMIC_GRASP_TRAJ (6), DYNAMIC_GRASP_POS (7), and FF_PID_TRAJ_MANIP (8). A description of these modes (with how these gains are implemented can be found there).



pc104.sendHomePos(home); %cmd = 7
This function tells the PC104 which cartesian or joint positions are the 'home' values. The home values are used often as a starting place for a trajectory. pc104.goHome() can be called to then tell the object to go home, and depending on the mode, GO_HOME_JOINTS (1) for joint positions or PID_MANIP_POS (9) / DYNAMIC_GRASP_POS (7) for cartesian positions, the manipulator will go to those positions and hold there indefinitely.



encPos = pc104.currentEncoderPos(); %cmd = 8
Reads in the joint positions from the PC104 and prints it out and returns it as an array. The array is 3 doubles [RH14 RH11 RH8]



pc104.stopAndReset(); %cmd = 9
This function tells the PC104 to STOP what it is doing, and reset as if it had just been turned on. This makes it possible to run multiple experiments in a row without too much time in between.




After connecting to the PC-104, it is good to reset the encoders.
pc104.resetEncoders(); %cmd = 10
This works by using the camera to calculate the joint angles of motor 1 and 2 (RH14 and RH11) while the third link is assumed to have the same orientation as the object. This can be changed in the C code on the PC104.



pc104.misc(); %cmd = 11
This function isnt really used. I was using it to do testing and stuff for various things and the function can be changed to do different testing. Right now the function doesn't do anything but is there in case it could be useful in the future.


Note that cmd = 12 is used by the PC104 to calculate the contact point locations of the block on the manipulator in dynamic grasp. I never ended up actually using this so its not implemented in the matlab class. It shouldn't be too difficult to implement if needed.


pc104.LQRGainSend(); %cmd = 13
This function sends out LQR gains based on the A,B,Q, and R matrices in the internal parameters struct. This is for the infinite-time case where the feedback matrix K is constant and so only 24 doubles are sent back (K is 3x8). Refer to ParametersFunction() to see how A and B are defined which indicates which variables are being used for feedback. I think in this case the state variables are: (xm, xmd, ym, ymd, thm, thmd, tho, thod). The controls are as always u = (xmdd, ymdd, thmdd). The modes used by this are ONE_POINT_ROLL_BALANCE (10).



pc104.newLQRGainSend(); %cmd = 13
Same as pc104.LQRGainSend() but uses Anew, Bnew, Qnew, and Rnew in ParametersFunction(). This one uses (thm, thmd, xo, xod, yo, yod, tho, thod). The mode that uses this is NEW_ONE_POINT_ROLL_BALANCE (11).


pc104.killProgram(); %cmd = 14
This function ends the program running on the PC104.

pc104.LQRTrajSend(traj1, traj2, traj3, traj4, uffx, uffy, uffth, t); %cmd = 15
Calculates the finite-time LQR problem for the given trajectory and controls. It then sends over the huge 3x8xnum_pts K matrix for each point in time as well as the full trajectory and control. This takes a long time but is usually the preferred method for one-point rolling as its the most general version. The problem with it is that it is quite slow. It is used with the control mode ONE_POINT_ROLL_TRAJ (12).

pc104.setTrajectoryControlMode(control_mode); %cmd = 16
Sends over the desired control mode for the following trajectory. This must be called before executing a trajectory or else the control mode will be set to NO_CONTROL (0). Control modes can be found in ParametersFunction() or the params struct (e.g. params.NO_CONTROL == 0)



pc104.plotTrajData();
This function is called after getting the trajectory data and plots it. The plotting opens a bunch of figure windows. Sometimes the data plotted doesn't quite match what you want depending on what the "traj" variables represent for example.



K = pc104.LQRTrajGains(traj1, traj2, traj3, traj4, uffx, uffy, uffth) 
Calculates the finite-time LQR feedback gain matrix as a function of time for the LQRTrajSend function.



pc104.sendDoubles(val), pc104.recDoubles(n_pts) should not be used by the end user and are lower-level calls to send data back and forth between the PC104. DO NOT EDIT THESE UNLESS YOU KNOW WHAT YOU ARE DOING. Messing with them could break a lot of code. They handle low-level sending and receiving of arrays of doubles.



PC104_Arm3DoF Class Fields:
port_in: port used for receiving TCP/IP
port_out: port used for sending TCP/IP
dt: Control loop timestep, almost always 0.001s
IPin: IP to receive TCP/IP from
IPout: IP to send TCP/IP to
params: the params struct initialized from ParametersFunction()
connected: a 1 if the QNX pc104 is connected, 0 else
num_pts: The number of points in the trajectory, determines the length of all vectors
traj1/2/3/4: arrays containing the desired trajectories. Could be in cartesian or joint space, for object or manipulator.
pos1/2/3: Joint positions for each motor/joint. 1/2/3 refers to joints 1/2/3 and motors RH14/RH11/RH8
controlVals1/2/3: The desired currents in Amps sent to the Junus motor controller.
camPosX/Y/Th: Position of the end effector of the manipulator as seen from the camera.
loopTimes: A vector telling us how long each loop took. Should all be much less than 0.001s to make sure we get accurate loop timing.
objPosX/Y/Th: Position of the object center of mass as seen by the camera
t: vector of times for the trajectories, gives a time scaling for the indices.
x/y/thAccControl: Desired accelerations of the end effector of the manipulator. We put this as our control for the higher-level controllers and assume we can control to this acceleration. In reality, we can't but we get close.
accControl1/2/3: Desired accelerations of joints 1/2/3 (motors RH14/11/8) which are calculated on the PC104 essentially using inverse dynamics.
k(1/2)RH(14/11/8): The "inner loop" control values for the pc104. This was used to improve acceleration control. The main idea is to integrate forward the desired accelerations into a position signal and apply PD control to the output of the higher level controller. This idea came from Ji-Chul and did improve acceleration control but its still not perfect. Every once in awhile the integrated position signal needs to be reset to account for integration error/drift.
tcam: a vector of times that map indices for camera vectors to real times. This needs to be different than t because the camera operates at 250Hz instead of 1kHz like the control loop.
fixedCamX/Y: Fixed camera arrays for the manipulator (note that manipulator Th can't be determined by the camera as it would need two markers). By fixed, we mean that "duplicate" values that are written at 1kHz and removed, and only "new" values written at 250Hz are kept.
fixedObjX/Y/Th: Same as fixedCamX/Y but for the object and also including Th since the object has two markers to track orientation.


************************************************************************
TEST SCRIPTS
************************************************************************


All test scripts can be found in the Manipulation/Experiment_Files/ directory along with the PC104_Arm3DoF.m matlab class file.

GravityTest.m
This script was used with the NO_CONTROL (0) mode to measure data from the cameras to calculate gravity. I would start this, then throw the block and let it fall, pulling all the data corresponding to parabolic motion and fit those to quadratics and use the quadratic term to calculate gravity (twice gravity to be precise). Then I would average it over all the parabolic motions in the 30s period and use that average. This corresponded to a table angle of 0.4 radians.


LimitCheckScript.m
Similar to the GravityTestScript.m in the sense that no control is used, but different in that we don't actually need to execute a trajectory. The goal of this script was not to record data, but just to quickly gather the joint angles when they are at their maximum and minimum. You start up this script, move the arm to a position where you want the joint angles, and unpause matlab and they will be printed to the screen or returned from the function.


currentTestJoint3.m
A script to demonstrate the TRAJ_IS_CURRENT (2) mode. This demonstrates how you could create trajectories which are currents (in Amps) for the individual motors and send them over.


OnePointRollingTrajTest.m
This script demonstrates the ONE_POINT_ROLL_TRAJ (12) mode. This shows how the trajectory is linearized and the LQR is calculated. Note this trajectory could be a constant.



