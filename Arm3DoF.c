/*
 * Arm3DoF.c
 *
 *  Created on: Oct 16, 2012
 *      Author: Adam
 */

#include "Arm3DoF.h"

void initialize_variables() {
	currRH14 = 0;
	currRH11 = 0;
	currRH8	 = 0;
	lastRH14 = 0;
	lastRH11 = 0;
	lastRH8  = 0;
	total_countRH14 = 0;
	total_countRH11 = 0;
	total_countRH8  = 0;
	error1 = 0; error2 = 0; error3 = 0;
	errord1 = 0; errord2 = 0; errord3 = 0;
	errorInt1 = 0; errorInt2 = 0; errorInt3 = 0;
	home1 = 0; home2 = 0; home3 = 0;
	running = 0;
	kp1 = 0; kp2 = 0; kp3 = 0;
	kd1 = 0; kd2 = 0; kd3 = 0;
	ki1 = 0; ki2 = 0; ki3 = 0;
	num_pts = 0;
	control_mode = 0;
	globalIndex = -1;
	DIO_word = 0x0000;
	longestLoopTime = 0;
}

void initialize_junus(uintptr_t iobase) {
	//First output 0 control to all motors
	set_control_RH8(iobase, 0.0);
	set_control_RH11(iobase, 0.0);
	set_control_RH14(iobase, 0.0);
	//Set pin 8 as output and high, it is the enable pin
	//Need to not change this when we write to DIO
	DIO_word = DIO_word | 0x0880;
	out16(iobase + DIO_ADDR, DIO_word);
}

double current_position_RH8(uintptr_t iobase, int reset){
	currRH8 = read_encoder_3(iobase);
	if(currRH8 >= lastRH8) {
		if(currRH8 - lastRH8 > 8388608)
			total_countRH8 -= 16777216 + lastRH8 - currRH8; //Negative rollover
		else //No Rollover
			total_countRH8 += currRH8 - lastRH8;
	} else {
		if(lastRH8 - currRH8 > 8388608)
			total_countRH8 += 16777216 + currRH8 - lastRH8; //Positive rollover
		else //No rollver
			total_countRH8 -= lastRH8 - currRH8;
	}

	lastRH8 = currRH8;
	if(reset) {
		total_countRH8 = 0;
	}
	//total_countRH8 = read_encoder_1(iobase);
//	return (((double)(total_countRH8))/180000.0)*M_PI;
	return (((double)(total_countRH8))/51200.0)*M_PI;
}

double current_position_RH11(uintptr_t iobase, int reset){
	currRH11 = read_encoder_2(iobase);
	if(currRH11 >= lastRH11) {
		if(currRH11 - lastRH11 > 8388608)
			total_countRH11 -= 16777216 + lastRH11 - currRH11; //Negative rollover
		else //No Rollover
			total_countRH11 += currRH11 - lastRH11;
	} else {
		if(lastRH11 - currRH11 > 8388608)
			total_countRH11 += 16777216 + currRH11 - lastRH11; //Positive rollover
		else //No rollver
			total_countRH11 -= lastRH11 - currRH11;
	}

	lastRH11 = currRH11;
	if(reset) {
		//Want reset to set it to -0.7977 (approx)
	    total_countRH11 = (int)(calculateJointTwoCamera()/M_PI*200000.0);
//		total_countRH11 = 0;
	}

	return (((double)(total_countRH11))/200000.0)*M_PI;
}

double current_position_RH14(uintptr_t iobase, int reset){
	currRH14 = read_encoder_1(iobase);
	if(currRH14 >= lastRH14) {
		if(currRH14 - lastRH14 > 8388608)
			total_countRH14 -= 16777216 + lastRH14 - currRH14; //Negative rollover
		else //No Rollover
			total_countRH14 += currRH14 - lastRH14;
	} else {
		if(lastRH14 - currRH14 > 8388608)
			total_countRH14 += 16777216 + currRH14 - lastRH14; //Positive rollover
		else //No rollver
			total_countRH14 -= lastRH14 - currRH14;
	}

	lastRH14 = currRH14;
	if(reset) {
		//Want reset to set it to 1.838 rads
//		total_countRH14 = (int)(-1.0*1.838/M_PI*100000.0);
//		total_countRH14 = 0;
	    total_countRH14 = (int)(calculateJointOneCamera()/M_PI*100000.0);
	}

	return -1.0*(((double)(total_countRH14))/100000.0)*M_PI;
}

void set_control_RH8(uintptr_t iobase, double desCurrent) {
	if(desCurrent >= MAX_CURRENT_RH8)
		desCurrent = MAX_CURRENT_RH8;
	else if(desCurrent <= -1.0*MAX_CURRENT_RH8)
		desCurrent = -1.0*MAX_CURRENT_RH8;

	uint16_t DACVal = (desCurrent/MAX_CURRENT_RH8 + 1.0)*32767.0;
	//Write to analog out 2 (3 on the breakout):
	out16(iobase + DAC_ADDR, 0x0004);
	//Load preregister for DAC0
	out16(iobase + ADD_ADDR, DACVal);
	//Start D/A conversion
	out16(iobase + DAC_ADDR, 0x0001);
	//printf("desired current: %f\n DACval: %u\n",desCurrent,DACVal);
}

void set_control_RH11(uintptr_t iobase, double desCurrent) {
	if(desCurrent >= MAX_CURRENT_RH11)
		desCurrent = MAX_CURRENT_RH11;
	else if(desCurrent <= -1.0*MAX_CURRENT_RH11)
		desCurrent = -1.0*MAX_CURRENT_RH11;

	uint16_t DACVal = (desCurrent/MAX_CURRENT_RH11 + 1.0)*32767.0;
	//Write to analog out 1 (2 on the breakout):
	out16(iobase + DAC_ADDR, 0x0002);
	//Load preregister for DAC0
	out16(iobase + ADD_ADDR, DACVal);
	//Start D/A conversion
	out16(iobase + DAC_ADDR, 0x0001);
	//printf("desired current: %f\n DACval: %u\n",desCurrent,DACVal);
}

void set_control_RH14(uintptr_t iobase, double desCurrent) {
	if(desCurrent >= MAX_CURRENT_RH14)
		desCurrent = MAX_CURRENT_RH14;
	else if(desCurrent <= -1.0*MAX_CURRENT_RH14)
		desCurrent = -1.0*MAX_CURRENT_RH14;

	uint16_t DACVal = (desCurrent/MAX_CURRENT_RH14 + 1.0)*32767.0;
	//Write to analog out 0 (1 on the breakout):
	out16(iobase + DAC_ADDR, 0x0000);
	//Load preregister for DAC0
	out16(iobase + ADD_ADDR, DACVal);
	//Start D/A conversion
	out16(iobase + DAC_ADDR, 0x0001);
	//printf("desired current: %f\n DACval: %u\n",desCurrent,DACVal);
}

//"Reset" by freeing variables, zeroing out old variables, and prepare for another run
void simpleReset() {
	//Turn off control, global index returns to -1
	globalIndex = -1;
	//Deallocate memory
	free(traj1);
	free(traj2);
	free(traj3);
	free(position1);
	free(position2);
	free(position3);
	free(controlVals1);
	free(controlVals2);
	free(controlVals3);
	free(loopTimes);
	free(cameraPosX);
	free(cameraPosY);
	free(cameraPos1);
	free(cameraPos2);
	//Ensure control mode is 0 and no control
	control_mode = 0;
	running = 0;
	//Reset errors
	error1 = 0;
	error2 = 0;
	error3 = 0;
	errord1 = 0;
	errord2 = 0;
	errord3 = 0;
	errorInt1 = 0;
	errorInt2 = 0;
	errorInt3 = 0;
	//Reset number of trajectory points
	num_pts = 0;

	//set control vals to 0 current
	set_control_RH8(iobase, 0);
	set_control_RH11(iobase, 0);
	set_control_RH14(iobase, 0);
	//Can send back timing stuff now
	char buf[100];
	sprintf(buf,"%ull\n",longestLoopTime);
	sendString(buf);
	longestLoopTime = 0;
	//Tell PC we reset
	sendString("RESET\n");
}

//Functions to calculate the necessary current/calculate control
//RH14, motor 1
double calculateControl1() {
	//Initialize variables to 0
	double reqCurrent = 0;
	//Required torque
	double torque = 0;
	//Finite differenced velocity
	double vel = 0;
	//Term for gravity compensation
	double gravComp = 0;

	//If we are running a trajectory, calculate velocity
	if(running && globalIndex >0 && globalIndex < num_pts) {
		vel = (position1[globalIndex] - position1[globalIndex - 1])/DT;
	}

	//Determine our control mode
	switch(control_mode) {
	case 0:
		//Don't do anything, use 0 current
		break;
	case 1:
		//Go home
		//Calculate current error
		error1 = home1 - current_position_RH14(iobase, 0);
		//Update integral error
		errorInt1 += error1*DT;
		gravComp = -0.5*g*(L1*(m1+2.0*(m2+m3+mm2+mm3))*cos(home1) + \
				L2*(m2+2.0*(m3+mm3))*cos(home1+home2));
		//Use PI + gravComp for home control
		torque = KPH1*error1 + KIH1*errorInt1 + gravComp;
		reqCurrent = torque/KM1;
		break;
	case 2:
		//Trajectories are actually current vals
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			reqCurrent = traj1[globalIndex];
		}
		break;
	case 3:
		//Feedforward control
		torque = feedForward1();
		reqCurrent = torque/KM1;
		break;
	case 4:
		//PID Traj control
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			error1 = traj1[globalIndex] - position1[globalIndex];
			errord1 = calculateTrajVel1() - vel;
			errorInt1 += error1*DT;
			torque = kp1*error1 + kd1*errord1 + ki1*errorInt1;
			reqCurrent = torque/KM1;
		}
		break;
	case 5:
		//Feedforward control with PID
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			error1 = traj1[globalIndex] - position1[globalIndex];
			errord1 = calculateTrajVel1() - vel;
			errorInt1 += error1*DT;
			torque = feedForward1() + kp1*error1 + kd1*errord1 + ki1*errorInt1;
			reqCurrent = torque/KM1;
		}
		break;
	default: break;
	}
	//Flip current, motor is installed "backwards"
	reqCurrent = -1.0*reqCurrent;
	//Setting the control will check bounds
	return reqCurrent;
}

//RH11, motor 2
double calculateControl2() {
	//Initialize variables to 0
	double reqCurrent = 0;
	//Required torque
	double torque = 0;
	//Finite differenced velocity
	double vel = 0;
	//Term for gravity compensation
	double gravComp = 0;

	//If we are running a trajectory, calculate velocity
	if(running && globalIndex >0 && globalIndex < num_pts) {
		vel = (position2[globalIndex] - position2[globalIndex - 1])/DT;
	}

	//Determine our control mode
	switch(control_mode) {
	case 0:
		//Don't do anything, use 0 current
		break;
	case 1:
		//Go home
		//Calculate current error
		error2 = home2 - current_position_RH11(iobase, 0);
		//Update integral error
		errorInt2 += error2*DT;
		gravComp = -0.5*g*L2*(m2+2.0*(m3+mm3))*cos(home1+home2);
		//Use PI + gravComp for home control
		torque = KPH2*error2 + KIH2*errorInt2 + gravComp;
		reqCurrent = torque/KM2;
		break;
	case 2:
		//Trajectories are actually current vals
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			reqCurrent = traj2[globalIndex];
		}
		break;
	case 3:
		//Feedforward control
		torque = feedForward2();
		reqCurrent = torque/KM2;
		break;
	case 4:
		//PID Traj control
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			error2 = traj2[globalIndex] - position2[globalIndex];
			errord2 = calculateTrajVel2() - vel;
			errorInt2 += error2*DT;
			torque = kp2*error2 + kd2*errord2 + ki2*errorInt2;
			reqCurrent = torque/KM2;
		}
		break;
	case 5:
		//Feedforward control with PID
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			error2 = traj2[globalIndex] - position2[globalIndex];
			errord2 = calculateTrajVel2() - vel;
			errorInt2 += error2*DT;
			torque = feedForward2() + kp2*error2 + kd2*errord2 + ki2*errorInt2;
			reqCurrent = torque/KM2;
		}
		break;
	default: break;
	}
	//Setting the control will check bounds
	return reqCurrent;
}

//RH8, motor 3
double calculateControl3() {
	//Initialize variables to 0
	double reqCurrent = 0;
	//Required torque
	double torque = 0;
	//Finite differenced velocity
	double vel = 0;

	//If we are running a trajectory, calculate velocity
	if(running && globalIndex >0 && globalIndex < num_pts) {
		vel = (position3[globalIndex] - position3[globalIndex - 1])/DT;
	}

	//Determine our control mode
	switch(control_mode) {
	case 0:
		//Don't do anything, use 0 current
		break;
	case 1:
		//Go home
		//Calculate current error
		error3 = home3 - current_position_RH8(iobase, 0);
		//Update integral error
		errorInt3 += error3*DT;
		//Use PI + gravComp for home control
		torque = KPH3*error3 + KIH3*errorInt3;
		reqCurrent = torque/KM3;
		break;
	case 2:
		//Trajectories are actually current vals
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			reqCurrent = traj3[globalIndex];
		}
		break;
	case 3:
		//Feedforward control
		torque = feedForward3();
		reqCurrent = torque/KM3;
		break;
	case 4:
		//PID Traj control
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			error3 = traj3[globalIndex] - position3[globalIndex];
			errord3 = calculateTrajVel3() - vel;
			errorInt3 += error3*DT;
			torque = kp3*error3 + kd3*errord3 + ki3*errorInt3;
			reqCurrent = torque/KM3;
		}
		break;
	case 5:
		//Feedforward control with PID
		if(globalIndex >= 0 && globalIndex < num_pts && running) {
			error3 = traj3[globalIndex] - position3[globalIndex];
			errord3 = calculateTrajVel3() - vel;
			errorInt3 += error3*DT;
			torque = feedForward3() + kp3*error3 + kd3*errord3 + ki3*errorInt3;
			reqCurrent = torque/KM3;
		}
		break;
	default: break;
	}
	//Setting the control will check bounds
	return reqCurrent;
}

double feedForward1() {
	double s1, s2, s12, c1, c2, c12, th1dot, th2dot, th1ddot, th2ddot, th3ddot, torqueDes, torqueFriction;
	double th1, th2;

	if(globalIndex >= 0 && globalIndex < num_pts && running) {
		th1 = traj1[globalIndex];
		th2 = traj2[globalIndex];
		s1 = sin(th1);
		s2 = sin(th2);
		s12 = sin(th1+th2);
		c1 = cos(th1);
		c2 = cos(th2);
		c12 = cos(th1+th2);
		th1dot = calculateTrajVel1();
		th2dot = calculateTrajVel2();
		th1ddot = calculateTrajVel1();
		th2ddot = calculateTrajVel2();
		th3ddot = calculateTrajVel3();

		torqueDes = -0.5*g*(L1*(m1+2.0*(m2 + m3 + mm2 + mm3))*c1 + \
				L2*(m2+2.0*(m3+mm3))*c12) - \
		        L1*L2*(m2+2.0*(m3+mm3))*s2*th1dot*th2dot - \
		        0.5*L1*L2*(m2+2.0*(m3+mm3))*s2*th2dot*th2dot + \
		        (I1 + I2 +I3 + L2*L2*(m2*0.25+m3+mm3)+L1*L1*(m1*.25+m2+m3+mm2+mm3) + \
		        J1+J2+J3 + L1*L2*(m2+2.0*(m3+mm3))*c2)*th1ddot + \
		        (I2+I3+L2*L2*(m2*0.25+m3+mm3) + J2 + J3+ \
		        0.5*L1*L2*(m2+2.0*(m3+mm3))*c2)*th2ddot + \
		        (I3+J3)*th3ddot;
		torqueFriction = MUD1*th1dot;
		if (th1dot > 0.0)
			torqueFriction += MUS1;
		else if(th1dot < 0.0)
			torqueFriction -= MUS1;
		return torqueDes + torqueFriction;
	} else
		return 0.0;
}

double feedForward2() {
	double th1, th2, s12, s2, c12, c2, th1dot, th2dot, th1ddot, th2ddot, th3ddot, torqueDes, torqueFriction;
	th1 = traj1[globalIndex];
	th2 = traj2[globalIndex];
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
		s12 = sin(th1 + th2);
		c12 = cos(th1 + th2);
		s2 = sin(th2);
		c2 = cos(th2);
		th1dot = calculateTrajVel1();
		th2dot = calculateTrajVel2();
		th1ddot = calculateTrajAccel1();
		th2ddot = calculateTrajAccel2();
		th3ddot = calculateTrajAccel3();
		torqueDes = 0.25*(-2.0*g*L2*(m2+2.0*(m3+mm3))*c12 + \
			2.0*L1*L2*(m2+2.0*(m3+mm3))*s2*th1dot*th1dot + \
		    (4.0*I2 + 4.0*I3 + L2*L2*(m2+4.0*(m3+mm3)) + 4.0*(J2+J3) + \
		    2.0*L1*L2*(m2+2.0*(m3+mm3))*c2)*th1ddot + \
		    (4.0*I2+4.0*I3+L2*L2*(m2+4.0*(m3+mm3))+4.0*(J2+J3))*th2ddot + \
		    4.0*(I3+J3)*th3ddot);

		torqueFriction = MUD2*th2dot;
		if(th2dot > 0.0)
			torqueFriction += MUS2;
		else if(th2dot < 0.0)
			torqueFriction -= MUS2;
		return torqueDes + torqueFriction;
	}
	else return 0.0;
}

double feedForward3() {
	double torqueDes, th3dot, torqueFriction;
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
		torqueDes = (I3+J3)*(calculateTrajAccel1() + calculateTrajAccel2() + calculateTrajAccel3());
		th3dot = calculateTrajVel3();
		torqueFriction = MUD3*th3dot;
		if(th3dot > 0.0)
			torqueFriction += MUS3;
		else if(th3dot < 0.0)
			torqueFriction -= MUS3;
		return torqueDes + torqueFriction;
	} else return 0.0;
}

double calculateTrajVel1(void) {
	if(globalIndex < 1) return 0.0;
	else {
		return (traj1[globalIndex] - traj1[globalIndex-1])/DT;
	}
}

double calculateTrajVel2(void) {
	if(globalIndex < 1) return 0.0;
	else {
		return (traj2[globalIndex] - traj2[globalIndex-1])/DT;
	}
}

double calculateTrajVel3(void) {
	if(globalIndex < 1) return 0.0;
	else {
		return (traj3[globalIndex] - traj3[globalIndex-1])/DT;
	}
}

double calculateTrajAccel1(void) {
	if(globalIndex < 1) return 0.0;
	else if(globalIndex >= num_pts -1) return 0.0;
	else {
		return(((traj1[globalIndex + 1] - traj1[globalIndex])/DT - (traj1[globalIndex] - traj1[globalIndex -1])/DT) /DT);
	}
}

double calculateTrajAccel2(void) {
	if(globalIndex < 1) return 0.0;
	else if(globalIndex >= num_pts -1) return 0.0;
	else {
		return(((traj2[globalIndex + 1] - traj2[globalIndex])/DT - (traj2[globalIndex] - traj2[globalIndex -1])/DT) /DT);
	}
}

double calculateTrajAccel3(void) {
	if(globalIndex < 1) return 0.0;
	else if(globalIndex >= num_pts -1) return 0.0;
	else {
		return(((traj3[globalIndex + 1] - traj3[globalIndex])/DT - (traj3[globalIndex] - traj3[globalIndex -1])/DT) /DT);
	}
}
