/*
 * Arm3DoF.c
 *
 *  Created on: Oct 16, 2012
 *      Author: Adam
 */

#include "Arm3DoF.h"

void initialize_variables() {
    //currRH14 = 0;
    //currRH11 = 0;
    //currRH8  = 0;
    //lastRH14 = 0;
    //lastRH11 = 0;
    //lastRH8  = 0;
    //total_countRH14 = 0;
    //total_countRH11 = 0;
    //total_countRH8  = 0;
//    error1 = 0; error2 = 0; error3 = 0;
//    errord1 = 0; errord2 = 0; errord3 = 0;
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
    contactPoint1 = -1.0;
    contactPoint2 = -2.0;
    thRH14global = 0.0;
    thRH11global = 0.0;
    thRH8global = 0.0;
    xManip_global = 0.0;
    yManip_global = 0.0;
    thManip_global = 0.0;
    velRH14global = 0.0;
    velRH11global = 0.0;
    velRH8global = 0.0;
    velXManip_global = 0.0;
    velYManip_global = 0.0;
    velThManip_global = 0.0;
    xObjectGlobal = 0.0;
    yObjectGlobal = 0.0;
    thObjectGlobal = 0.0;
    velXObjectGlobal = 0.0;
    velYObjectGlobal = 0.0;
    velThObjectGlobal = 0.0;
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
    static long long total_count = 0;
    static long curr = 0;
    static long last = 0;
    curr = read_encoder_3(iobase);
    if(curr >= last) {
	if(curr - last > 8388608)
	    total_count -= 16777216 + last - curr; //Negative rollover
	else //No Rollover
	    total_count += curr - last;
    } else {
	if(last - curr > 8388608)
	    total_count += 16777216 + curr - last; //Positive rollover
	else //No rollver
	    total_count -= last - curr;
    }

    last = curr;
    if(reset) {
//	total_count = (int)((thObjectGlobal-calculateJointTwoCamera() - calculateJointOneCamera())/M_PI*51200.0);
	total_count = (int)((-calculateJointTwoCamera() - calculateJointOneCamera())/M_PI*51200.0);
//		total_count = 0;
    }
    //total_count = read_encoder_1(iobase);
//	return (((double)(total_count))/180000.0)*M_PI;
    return (((double)(total_count))/51200.0)*M_PI;
}

double current_position_RH11(uintptr_t iobase, int reset){
    static long long total_count = 0;
    static long curr = 0;
    static long last = 0;
    curr = read_encoder_2(iobase);
    if(curr >= last) {
	if(curr - last > 8388608)
	    total_count -= 16777216 + last - curr; //Negative rollover
	else //No Rollover
	    total_count += curr - last;
    } else {
	if(last - curr > 8388608)
	    total_count += 16777216 + curr - last; //Positive rollover
	else //No rollver
	    total_count -= last - curr;
    }

    last = curr;
    if(reset) {
	//Want reset to set it to -0.7977 (approx)
	total_count = (int)(calculateJointTwoCamera()/M_PI*200000.0);
//		total_count = 0;
    }

    return (((double)(total_count))/200000.0)*M_PI;
}

double current_position_RH14(uintptr_t iobase, int reset){
    static long long total_count = 0;
    static long curr = 0;
    static long last = 0;
    curr = read_encoder_1(iobase);
    if(curr >= last) {
	if(curr - last > 8388608)
	    total_count -= 16777216 + last - curr; //Negative rollover
	else //No Rollover
	    total_count += curr - last;
    } else {
	if(last - curr > 8388608)
	    total_count += 16777216 + curr - last; //Positive rollover
	else //No rollver
	    total_count -= last - curr;
    }

    last = curr;
    if(reset) {
	//Want reset to set it to 1.838 rads
//		total_count = (int)(-1.0*1.838/M_PI*100000.0);
//		total_count = 0;
	total_count = (int)(-1.0*calculateJointOneCamera()/M_PI*100000.0);
    }

    return -1.0*(((double)(total_count))/100000.0)*M_PI;
}

void set_control_RH8(uintptr_t iobase, double desCurrent) {
	uint16_t DACVal;

    if(desCurrent >= (MAX_CURRENT_RH8 - MAX_CURRENT_RH8/32767.5)) {
    	DACVal = 65535;
    } else if((-1.0*desCurrent) >= (MAX_CURRENT_RH8 - MAX_CURRENT_RH8/32767.5)) {
    	DACVal = 0;
    } else {
    	DACVal = (uint16_t)((desCurrent/MAX_CURRENT_RH8 + 1.0)*32767.5);
    }

    //Write to analog out 2 (3 on the breakout):
    out16(iobase + DAC_ADDR, 0x0004);
    //Load preregister for DAC0
    out16(iobase + ADD_ADDR, DACVal);
    //Start D/A conversion
    out16(iobase + DAC_ADDR, 0x0001);
    //printf("desired current: %f\n DACval: %u\n",desCurrent,DACVal);
}

void set_control_RH11(uintptr_t iobase, double desCurrent) {
	uint16_t DACVal;
    if(desCurrent >= (MAX_CURRENT_RH11 - MAX_CURRENT_RH11/32767.5)) {
    	DACVal = 65535;
    } else if((-1.0*desCurrent) >= (MAX_CURRENT_RH11 - MAX_CURRENT_RH11/32767.5)) {
    	DACVal = 0;
    } else {
    	DACVal = (uint16_t)((desCurrent/MAX_CURRENT_RH11 + 1.0)*32767.5);
    }
    //Write to analog out 1 (2 on the breakout):
    out16(iobase + DAC_ADDR, 0x0002);
    //Load preregister for DAC0
    out16(iobase + ADD_ADDR, DACVal);
    //Start D/A conversion
    out16(iobase + DAC_ADDR, 0x0001);
    //printf("desired current: %f\n DACval: %u\n",desCurrent,DACVal);
}

void set_control_RH14(uintptr_t iobase, double desCurrent) {
	//Flip des current because motor is installed "backwards"
	desCurrent = -1.0*desCurrent;
	uint16_t DACVal;
    if(desCurrent >= (MAX_CURRENT_RH14 - MAX_CURRENT_RH14/32767.5)) {
    	DACVal = 65535;
    } else if((-1.0*desCurrent) >= (MAX_CURRENT_RH14 - MAX_CURRENT_RH14/32767.5)) {
    	DACVal = 0;
    } else {
    	DACVal = (uint16_t)((desCurrent/MAX_CURRENT_RH14 + 1.0)*32767.5);
    }
    //Debugging nonsensical stuff
//    if(control_mode == FF_PID_TRAJ_MANIP) {
//    	printf("curr: %f, dacval: %u\n",desCurrent,DACVal);
//    }
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
    free(cameraPosTh);
    free(cameraPos1);
    free(cameraPos2);
    //Ensure control mode is 0 and no control
    control_mode = NO_CONTROL;
    running = 0;
    //Reset errors
    //error1 = 0;
    //error2 = 0;
    //error3 = 0;
    //errord1 = 0;
    //errord2 = 0;
    //errord3 = 0;
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
    longestLoopTime = 0;
    //Tell PC we reset
    sendString("RESET\n");
}

//Calculate control for all three motors
void calculateControl(double *reqCurrentRH14, double *reqCurrentRH11, double *reqCurrentRH8){
	static int printTimer = 0;
    //Initialize variables
    *reqCurrentRH14 = 0.0; *reqCurrentRH11 = 0.0; *reqCurrentRH8 = 0.0;
    //Desired accelerations
    double th1dd, th2dd, th3dd;
    //Manipulator or object accelerations
    double xmdd, ymdd, thmdd;
    double xodd, yodd, thodd;
    //Motor torques desired
    double torqueDes1, torqueDes2, torqueDes3;
    //Errors for joints 1 2 and 3
    double error1, error2, error3;
    double errord1, errord2, errord3;
    //errors in x, y, th
    double errorX, errorY, errorTh;
    double errorXd, errorYd, errorThd;

    double errorThm, errorThmDot;
    double errorXo, errorXoDot;
    double errorYo, errorYoDot;
    double errorTho, errorThoDot;
    
//    static double th1Prev = 0.0;
//    static double th2Prev = 0.0;
//    static double th3Prev = 0.0;

//    static double th1DotCmd_prev = 0.0;
//    static double th1Cmd_prev = 0.0;
//    static double th2DotCmd_prev = 0.0;
//    static double th2Cmd_prev = 0.0;
//    static double th3DotCmd_prev = 0.0;
//    static double th3Cmd_prev = 0.0;
//    double th1Cmd, th1DotCmd, th2Cmd, th2DotCmd, th3Cmd, th3DotCmd;
//    if(globalIndex == 0) {
//    	th1Cmd_prev = traj1[globalIndex];
//    	th2Cmd_prev = traj2[globalIndex];
//    	th3Cmd_prev = traj3[globalIndex];
//    	th1DotCmd_prev = (traj1[globalIndex+1] - traj1[globalIndex])/DT;
//    	th2DotCmd_prev = (traj2[globalIndex+1] - traj2[globalIndex])/DT;
//    	th3DotCmd_prev = (traj3[globalIndex+1] - traj3[globalIndex])/DT;
//    }
    //Switch based on control_mode
    switch(control_mode) {
    case NO_CONTROL:
	//Don't do anything, 0 current
//	th1Prev = thRH14global;
//	th2Prev = thRH11global;
//	th3Prev = thRH8global;
	return;
	break;
    case GO_HOME_JOINTS:
	//Go home in joint space
	//calculate current errors
	error1 = home1 - thRH14global;
	error2 = home2 - thRH11global;
	error3 = home3 - thRH8global;
	errord1 = -1.0*velRH14global;
	errord2 = -1.0*velRH11global;
	errord3 = -1.0*velRH8global;
	errorInt1 += error1*DT;
	errorInt2 += error2*DT;
	errorInt3 += error3*DT;
	//Now set desired accelerations
	th1dd = kp1*error1 + kd1*errord1 + ki1*errorInt1;
	th2dd = kp2*error2 + kd2*errord2 + ki2*errorInt2;
	th3dd = kp3*error3 + kd3*errord3 + ki3*errorInt3;
//	//Only for this test
//	robotTorques(&torqueDes1, &torqueDes2, &torqueDes3,		 \
//			 th1dd, th2dd, th3dd,				 \
//			 velRH14global, velRH11global, velRH8global,	 \
//			 thRH14global, thRH11global, thRH8global);
//	*reqCurrentRH14 = torqueDes1/KM1;
//	*reqCurrentRH11 = torqueDes2/KM2;
//	*reqCurrentRH8 = torqueDes3/KM3;
//	th1Prev = thRH14global;
//	th2Prev = thRH11global;
//	th3Prev = thRH8global;
//	return;
	break;
    case TRAJ_IS_CURRENT:
//	th1Prev = thRH14global;
//	th2Prev = thRH11global;
//	th3Prev = thRH8global;
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
	    *reqCurrentRH14 = traj1[globalIndex];
	    *reqCurrentRH11 = traj2[globalIndex];
	    *reqCurrentRH8  = traj3[globalIndex];
	}
	return;
	break;
    case FEEDFORWARD_JOINTS:
	//Not sure we need it
	th1dd = calculateTrajAccel1();
	th2dd = calculateTrajAccel2();
	th3dd = calculateTrajAccel3();
	break;
    case PID_TRAJ_JOINTS:
	error1 = traj1[globalIndex] - thRH14global;
	error2 = traj2[globalIndex] - thRH11global;
	error3 = traj3[globalIndex] - thRH8global;
	errord1 = -velRH14global;
	errord2 = -velRH11global;
	errord3 = -velRH8global;
	errorInt1 += error1*DT;
	errorInt2 += error2*DT;
	errorInt3 += error3*DT;
	th1dd = kp1*error1 + kd1*errord1 + ki1*errorInt1;
	th2dd = kp2*error2 + kd2*errord2 + ki2*errorInt2;
	th3dd = kp3*error3 + kd3*errord3 + ki3*errorInt3;
	break;
    case FF_PID_TRAJ_JOINTS:
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
	    error1 = traj1[globalIndex] - thRH14global;
	    error2 = traj2[globalIndex] - thRH11global;
	    error3 = traj3[globalIndex] - thRH8global;
	    errord1 = calculateTrajVel1() - velRH14global;
	    errord2 = calculateTrajVel2() - velRH11global;
	    errord3 = calculateTrajVel3() - velRH8global;
	    errorInt1 += error1*DT;
	    errorInt2 += error2*DT;
	    errorInt3 += error3*DT;
	    th1dd = calculateTrajAccel1() + kp1*error1+kd1*errord1+ki1*errorInt1;
	    th2dd = calculateTrajAccel2() + kp2*error2+kd2*errord2+ki2*errorInt2;
	    th3dd = calculateTrajAccel3() + kp3*error3+kd3*errord3+ki3*errorInt3;
//	    desAccel1[globalIndex] = th1dd;
//	    desAccel2[globalIndex] = th2dd;
//	    desAccel3[globalIndex] = th3dd;
	} else return;
	break;
    case DYNAMIC_GRASP_TRAJ:
	//Dynamic grasp trajectory following:
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
	    errorX = traj1[globalIndex] - xObjectGlobal;
	    errorY = traj2[globalIndex] - yObjectGlobal;
	    errorTh = traj3[globalIndex] - thObjectGlobal;
	    errorXd = calculateTrajVel1() - velXObjectGlobal;
	    errorYd = calculateTrajVel2() - velYObjectGlobal;
	    errorThd = calculateTrajVel3() - velThObjectGlobal;
	    //Min distance on circle
	    if(fabs(errorTh) > fabs(2.0*M_PI - errorTh)) {
		errorTh = 2.0*M_PI - errorTh;
	    } else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) {
		errorTh = 2.0*M_PI + errorTh;
	    }
	    //calculate desired object acceleration
	    xodd = calculateTrajAccel1() + errorX*kp1 + errorXd*kd1;
	    yodd = calculateTrajAccel2() + errorY*kp2 + errorYd*kd2;
	    thodd = calculateTrajAccel3() + errorTh*kp3 + errorThd*kd3;
	    dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd);
	    calculateJointAccelFromManipAccel(xmdd,ymdd,thmdd, &th1dd, &th2dd, &th3dd);
	}
	break;
    case DYNAMIC_GRASP_POS:
	errorX = home1 - xObjectGlobal;
	errorY = home2 - yObjectGlobal;
	errorTh = home3 - thObjectGlobal;
	errorXd =  -1.0*velXObjectGlobal;
	errorYd =  -1.0*velYObjectGlobal;
	errorThd = -1.0*velThObjectGlobal;
	//Min distance on circle
	if(fabs(errorTh) > fabs(2.0*M_PI - errorTh)) {
	    errorTh = 2.0*M_PI - errorTh;
	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) {
	    errorTh = 2.0*M_PI + errorTh;
	}
	//calculate desired object acceleration
	xodd =  45.0*errorX + 25.0*errorXd;//errorX*kp1 + errorXd*kd1;
	yodd =  45.0*errorY + 25.0*errorYd;//errorY*kp2 + errorYd*kd2;
	thodd = 45.0*errorTh + 20.0*errorThd;//errorTh*kp3 + errorThd*kd3;
	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd);
	calculateJointAccelFromManipAccel(xmdd,ymdd,thmdd, &th1dd, &th2dd, &th3dd);
	break;
    case FF_PID_TRAJ_MANIP:
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
	    error1 = traj1[globalIndex] - xManip_global;
	    error2 = traj2[globalIndex] - yManip_global;
	    error3 = traj3[globalIndex] - thManip_global;
	    if(fabs(error3) > fabs(2*M_PI - error3)) {
		error3 = 2.0*M_PI - error3;
	    } else if(fabs(error3) > fabs(2*M_PI + error3)) {
		error3 = 2.0*M_PI + error3;
	    }
	    errord1 = calculateTrajVel1() - velXManip_global;
	    errord2 = calculateTrajVel2() - velYManip_global;
	    errord3 = calculateTrajVel3() - velThManip_global;
	    errorInt1 += error1;
	    errorInt2 += error2;
	    errorInt3 += error3;
	    xmdd = calculateTrajAccel1() + kp1*error1 + kd1*errord1 + ki1*errorInt1;
	    ymdd = calculateTrajAccel2() + kp2*error2 + kd2*errord2 + ki2*errorInt2;
	    thmdd = calculateTrajAccel3() + kp3*error3 + kd3*errord3 + ki3*errorInt3;
	    calculateJointAccelFromManipAccel(xmdd, ymdd, thmdd, &th1dd, &th2dd, &th3dd);
//	    desAccel1[globalIndex] = th1dd;
//	    desAccel2[globalIndex] = th2dd;
//	    desAccel3[globalIndex] = th3dd;
	} else return;
	break;
    case PID_MANIP_POS:
	error1 = home1 - xManip_global;
	error2 = home2 - yManip_global;
	error3 = home3 - thManip_global;	
	if(fabs(error3) > fabs(2*M_PI - error3)) {
	    error3 = 2.0*M_PI - error3;
	} else if(fabs(error3) > fabs(2*M_PI + error3)) {
	    error3 = 2.0*M_PI + error3;
	}
	errord1 = -1.0*velXManip_global;
	errord2 = -1.0*velYManip_global;
	errord3 = -1.0*velThManip_global;
	errorInt1 += error1;
	errorInt2 += error2;
	errorInt3 += error3;
	xmdd = 265.0*error1 + 100.0*errord1 + 0.0001*errorInt1;
	ymdd = 265.0*error2 + 100.0*errord2 + 0.0001*errorInt2;
	thmdd = 1000.0*error3 + 125.0*errord3 + 0.0001*errorInt3;
	calculateJointAccelFromManipAccel(xmdd, ymdd, thmdd, &th1dd, &th2dd, &th3dd);
	break;
    case ONE_POINT_ROLL_BALANCE:
	//LQR errors
	errorThm = -thManip_global;
	errorThmDot = -velThManip_global;
	errorXo = X_OBJ_ROLL_GOAL - xObjectGlobal;
	errorXoDot = -velXObjectGlobal;
	errorYo = Y_OBJ_ROLL_GOAL - yObjectGlobal;
	errorYoDot = -velYObjectGlobal;
	errorTho = M_PI/2.0 - OBJECT_ANGLE - thObjectGlobal;
	errorThoDot = -velThObjectGlobal;
	xmdd = K_lqr[0]*errorThm + K_lqr[1]*errorThmDot + K_lqr[2]*errorXo + K_lqr[3]*errorXoDot + \
			K_lqr[4]*errorYo + K_lqr[5]*errorYoDot + K_lqr[6]*errorTho + K_lqr[7]*errorThoDot;
	ymdd = K_lqr[8]*errorThm + K_lqr[9]*errorThmDot + K_lqr[10]*errorXo + K_lqr[11]*errorXoDot + \
			K_lqr[12]*errorYo + K_lqr[13]*errorYoDot + K_lqr[14]*errorTho + K_lqr[15]*errorThoDot;
	thmdd = K_lqr[16]*errorThm + K_lqr[17]*errorThmDot + K_lqr[18]*errorXo + K_lqr[19]*errorXoDot + \
			K_lqr[20]*errorYo + K_lqr[21]*errorYoDot + K_lqr[22]*errorTho + K_lqr[23]*errorThoDot;
	if(ymdd < -0.75*g) {
		printf("ClippingControl");
		ymdd = -0.75*g;
	}
	calculateJointAccelFromManipAccel(xmdd,ymdd,thmdd,&th1dd,&th2dd,&th3dd);
	break;
    default:
	return;
	break;
    }
    robotTorques(&torqueDes1, &torqueDes2, &torqueDes3,		 \
		 th1dd, th2dd, th3dd,				 \
		 velRH14global, velRH11global, velRH8global,	 \
		 thRH14global, thRH11global, thRH8global);

    *reqCurrentRH14 = torqueDes1/KM1;
    *reqCurrentRH11 = torqueDes2/KM2;
    *reqCurrentRH8 = torqueDes3/KM3;

//    th1Prev = thRH14global;
//    th2Prev = thRH11global;
//    th3Prev = thRH8global;
}

/* //Functions to calculate the necessary current/calculate control */
/* //RH14, motor 1 */
/* double calculateControl1() { */
/*     //Initialize variables to 0 */
/*     double reqCurrent = 0; */
/*     //Required torque */
/*     double torque = 0; */
/*     //Term for gravity compensation */
/*     double gravComp = 0; */

/*     double xmdd, ymdd, thmdd; */
/*     double xodd, yodd, thodd; */
/*     double errorX, errorY, errorTh; */

/*     double th1dd; */
/*     static double th1Prev = 0.0; */
/*     //Determine our control mode */
/*     switch(control_mode) { */
/*     case NO_CONTROL: */
/* 	//Don't do anything, use 0 current */
/* 	break; */
/*     case GO_HOME_JOINTS: */
/* 	//Go home */
/* 	//Calculate current error */
/* 	error1 = home1 - thRH14global; */
/* 	//Update integral error */
/* 	errorInt1 += error1*DT; */
/* 	gravComp = -0.5*g*(L1*(m1+2.0*(m2+m3+mm2+mm3))*cos(home1) + \ */
/* 			   L2*(m2+2.0*(m3+mm3))*cos(home1+home2)); */
/* 	//Use PI + gravComp for home control */
/* 	torque = KPH1*error1 + KIH1*errorInt1 + gravComp; */
/* 	reqCurrent = torque/KM1; */
/* 	break; */
/*     case TRAJ_IS_CURRENT: */
/* 	//Trajectories are actually current vals */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    reqCurrent = traj1[globalIndex]; */
/* 	} */
/* 	break; */
/*     case FEEDFORWARD_JOINTS: */
/* 	//Feedforward control */
/* 	torque = feedForward1(); */
/* 	reqCurrent = torque/KM1; */
/* 	break; */
/*     case PID_TRAJ_JOINTS: */
/* 	//PID Traj control */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error1 = traj1[globalIndex] - thRH14global; */
/* 	    errord1 = calculateTrajVel1() - velRH14global; */
/* 	    errorInt1 += error1*DT; */
/* 	    torque = kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	    reqCurrent = torque/KM1; */
/* 	} */
/* 	break; */
/*     case FF_PID_TRAJ_JOINTS: */
/* 	//Feedforward control with PID */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error1 = traj1[globalIndex] - thRH14global; */
/* 	    errord1 = calculateTrajVel1() - velRH14global; */
/* 	    errorInt1 += error1*DT; */
/* 	    th1dd =  kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	    torque = feedForward1(th1dd); */
/* 	    reqCurrent = torque/KM1; */
/* 	} */
/* 	break; */
/*     case DYNAMIC_GRASP_TRAJ: */
/* 	//Dyanmic grasp */
/* 	errorX = traj1[globalIndex] - xObjectGlobal; */
/* 	errorY = traj2[globalIndex] - yObjectGlobal; */
/* 	errorTh = traj3[globalIndex] - thObjectGlobal; */
/* 	//Check for minimum distance on circle */
/* 	if(fabs(errorTh) > fabs(2*M_PI - errorTh)) { */
/* 	    errorTh = 2.0*M_PI - errorTh; */
/* 	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) { */
/* 	    errorTh = 2.0*M_PI + errorTh; */
/* 	} */
/* 	//Add code here to go from errors in OBJECT accel to manip accel */
/* 	xodd = calculateTrajAccel1() + errorX*kp1; */
/* 	yodd = calculateTrajAccel2() + errorY*kp2; */
/* 	thodd = calculateTrajAccel3() + errorTh*kp3; */
/* 	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd); */
/* 	th1dd = calculateJointAccelFromManipAccel1(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel1(xmdd, ymdd, thmdd); */
/* //	reqCurrent = torque/KM1 + kp1curr*((thRH14global - th1Prev) + 0.5*DT*DT*th1dd) + kd1curr*(DT*th1dd); */
/* 	break; */
/*     case DYNAMIC_GRASP_POS: */
/* 	//Hold a pose in dynamic grasp */
/* 	errorX = objHomeX - xObjectGlobal; */
/* 	errorY = objHomeY - yObjectGlobal; */
/* 	errorTh = objHomeTh - thObjectGlobal; */
/* 	if(fabs(errorTh) > fabs(2*M_PI - errorTh)) { */
/* 	    errorTh = 2.0*M_PI - errorTh; */
/* 	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) { */
/* 	    errorTh = 2.0*M_PI + errorTh; */
/* 	} */
/* 	xodd = errorX*kp1; */
/* 	yodd = errorY*kp2; */
/* 	thodd = errorTh*kp3; */
/* 	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd); */
/* 	th1dd = calculateJointAccelFromManipAccel1(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel1(xmdd, ymdd, thmdd); */
/* //	reqCurrent = torque/KM1 + kp1curr*((thRH14global - th1Prev) + 0.5*DT*DT*th1dd) + kd1curr*(DT*th1dd); */
/* 	break; */
/*     case FF_PID_TRAJ_MANIP: */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error1 = traj1[globalIndex] - xManip_global; */
/* 	    error2 = traj2[globalIndex] - yManip_global; */
/* 	    error3 = traj3[globalIndex] - thManip_global; */
/* 	    if(fabs(error3) > fabs(2*M_PI - error3)) { */
/* 		error3 = 2.0*M_PI - error3; */
/* 	    } else if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 		error3 = 2.0*M_PI + error3; */
/* 	    } */
/* 	    errord1 = calculateTrajVel1() - velXManip_global; */
/* 	    errord2 = calculateTrajVel2() - velYManip_global; */
/* 	    errord3 = calculateTrajVel3() - velThManip_global; */
/* 	    errorInt1 += error1; */
/* 	    errorInt2 += error2; */
/* 	    errorInt3 += error3; */
/* 	    xmdd = calculateTrajAccel1() + kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	    ymdd = calculateTrajAccel2() + kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	    thmdd = calculateTrajAccel3() + kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	    th1dd = calculateJointAccelFromManipAccel1(xmdd,ymdd,thmdd); */
/* 	    torque = controlManipAccel1(xmdd, ymdd, thmdd); */
/* 	    //For determining inner loop gains */
/* //	    desAccel1[globalIndex] = th1dd; */
/* //	    reqCurrent = torque/KM1 + kp1curr*((thRH14global - th1Prev) + 0.5*DT*DT*th1dd) + kd1curr*(DT*th1dd); */
/* 	} else reqCurrent = 0.0; */
/* 	break; */
/*     case PID_MANIP_POS: */
/* 	error1 = home1 - xManip_global; */
/* 	error2 = home2 - yManip_global; */
/* 	error3 = home3 - thManip_global; */
/* 	if(fabs(error3) > fabs(2*M_PI - error3)) { */
/* 	    error3 = 2.0*M_PI - error3; */
/* 	} else if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 	    error3 = 2.0*M_PI + error3; */
/* 	} */
/* 	errord1 = -1.0*velXManip_global; */
/* 	errord2 = -1.0*velYManip_global; */
/* 	errord3 = -1.0*velThManip_global; */
/* 	errorInt1 += error1; */
/* 	errorInt2 += error2; */
/* 	errorInt3 += error3; */
/* 	xmdd = kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	ymdd = kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	thmdd = kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	th1dd = calculateJointAccelFromManipAccel1(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel1(xmdd,ymdd,thmdd); */
/* //	reqCurrent = torque/KM1 + kp1curr*((thRH14global - th1Prev) + 0.5*DT*DT*th1dd) + kd1curr*(DT*th1dd); */
/* 	break; */
/*     default: break; */
/*     } */
/*     th1Prev = thRH14global; */
/*     //Setting the control will check bounds, and flip due to motor installation */
/*     return reqCurrent; */
/* } */

/* //RH11, motor 2 */
/* double calculateControl2() { */
/*     //Initialize variables to 0 */
/*     double reqCurrent = 0; */
/*     //Required torque */
/*     double torque = 0; */
/*     //Term for gravity compensation */
/*     double gravComp = 0; */

/*     //For dynamic grasp */
/*     double xmdd, ymdd, thmdd; */
/*     double xodd, yodd, thodd; */
/*     double errorX, errorY, errorTh; */

/*     static double th2Prev = 0.0; */

/*     double th2dd; */
/*     //Determine our control mode */
/*     switch(control_mode) { */
/*     case NO_CONTROL: */
/* 	//Don't do anything, use 0 current */
/* 	break; */
/*     case GO_HOME_JOINTS: */
/* 	//Go home */
/* 	//Calculate current error */
/* 	error2 = home2 - thRH11global; */
/* 	//Update integral error */
/* 	errorInt2 += error2*DT; */
/* 	gravComp = -0.5*g*L2*(m2+2.0*(m3+mm3))*cos(home1+home2); */
/* 	//Use PI + gravComp for home control */
/* 	torque = KPH2*error2 + KIH2*errorInt2 + gravComp; */
/* 	reqCurrent = torque/KM2; */
/* 	break; */
/*     case TRAJ_IS_CURRENT: */
/* 	//Trajectories are actually current vals */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    reqCurrent = traj2[globalIndex]; */
/* 	} */
/* 	break; */
/*     case FEEDFORWARD_JOINTS: */
/* 	//Feedforward control */
/* 	torque = feedForward2(); */
/* 	reqCurrent = torque/KM2; */
/* 	break; */
/*     case PID_TRAJ_JOINTS: */
/* 	//PID Traj control */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error2 = traj2[globalIndex] - thRH11global; */
/* 	    errord2 = calculateTrajVel2() - velRH11global; */
/* 	    errorInt2 += error2*DT; */
/* 	    torque = kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	    reqCurrent = torque/KM2; */
/* 	} */
/* 	break; */
/*     case FF_PID_TRAJ_JOINTS: */
/* 	//Feedforward control with PID */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error2 = traj2[globalIndex] - thRH8global; */
/* 	    errord2 = calculateTrajVel2() - velRH8global; */
/* 	    errorInt2 += error2*DT; */
/* 	    torque = feedForward2() + kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	    reqCurrent = torque/KM2; */
/* 	} */
/* 	break; */
/*     case DYNAMIC_GRASP_TRAJ: */
/* 	//Dyanmic grasp */
/* 	//Check for minimum distance on circle */
/* 	errorX = traj1[globalIndex] - xObjectGlobal; */
/* 	errorY = traj2[globalIndex] - yObjectGlobal; */
/* 	errorTh = traj3[globalIndex] - thObjectGlobal; */
/* 	if(fabs(errorTh) > fabs(2*M_PI - errorTh)) { */
/* 	    errorTh = 2.0*M_PI - errorTh; */
/* 	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) { */
/* 	    errorTh = 2.0*M_PI + errorTh; */
/* 	}	  //Add code here to go from errors in OBJECT accel to manip accel */
/* 	xodd = calculateTrajAccel1() + errorX*kp1; */
/* 	yodd = calculateTrajAccel2() + errorY*kp2; */
/* 	thodd = calculateTrajAccel3() + errorTh*kp3; */
/* 	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd); */
/* 	th2dd = calculateJointAccelFromManipAccel2(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel2(xmdd, ymdd, thmdd); */
/* //	reqCurrent = torque/KM2 + kp2curr*((thRH11global - th2Prev) + 0.5*DT*DT*th2dd) + kd2curr*(DT*th2dd); */
/* 	break; */
/*     case DYNAMIC_GRASP_POS: */
/* 	//Hold a pose in dynamic grasp */
/* 	errorX = objHomeX - xObjectGlobal; */
/* 	errorY = objHomeY - yObjectGlobal; */
/* 	errorTh = objHomeTh - thObjectGlobal; */
/* 	if(fabs(errorTh) > fabs(2*M_PI - errorTh)) { */
/* 	    errorTh = 2.0*M_PI - errorTh; */
/* 	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) { */
/* 	    errorTh = 2.0*M_PI + errorTh; */
/* 	} */
/* 	xodd = errorX*kp1; */
/* 	yodd = errorY*kp2; */
/* 	thodd = errorTh*kp3; */
/* 	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd); */
/* 	th2dd = calculateJointAccelFromManipAccel2(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel2(xmdd, ymdd, thmdd); */
/* //	reqCurrent = torque/KM2 + kp2curr*((thRH11global - th2Prev)+0.5*DT*DT*th2dd) + kd2curr*(DT*th2dd); */
/* 	break; */
/*     case FF_PID_TRAJ_MANIP: */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error1 = traj1[globalIndex] - xManip_global; */
/* 	    error2 = traj2[globalIndex] - yManip_global; */
/* 	    error3 = traj3[globalIndex] - thManip_global; */
/* 	    if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 		error3 = 2.0*M_PI - error3; */
/* 	    } else if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 		error3 = 2.0*M_PI + error3; */
/* 	    } */
/* 	    errord1 = calculateTrajVel1() - velXManip_global; */
/* 	    errord2 = calculateTrajVel2() - velYManip_global; */
/* 	    errord3 = calculateTrajVel3() - velThManip_global; */
/* 	    errorInt1 += error1; */
/* 	    errorInt2 += error2; */
/* 	    errorInt3 += error3; */
/* 	    xmdd = calculateTrajAccel1() + kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	    ymdd = calculateTrajAccel2() + kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	    thmdd = calculateTrajAccel3() + kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	    th2dd = calculateJointAccelFromManipAccel2(xmdd, ymdd, thmdd); */
/* //	    desAccel2[globalIndex] = th2dd; */
/* 	    torque = controlManipAccel2(xmdd, ymdd, thmdd); */
/* //	    reqCurrent = torque/KM2 + kp2curr*((thRH11global - th2Prev)+0.5*DT*DT*th2dd) + kd2curr*(DT*th2dd); */
/* 	} else reqCurrent = 0.0; */
/* 	break; */
/*     case PID_MANIP_POS: */
/* 	error1 = home1 - xManip_global; */
/* 	error2 = home2 - yManip_global; */
/* 	error3 = home3 - thManip_global; */
/* 	if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 	    error3 = 2.0*M_PI - error3; */
/* 	} else if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 	    error3 = 2.0*M_PI + error3; */
/* 	} */
/* 	errord1 = -1.0*velXManip_global; */
/* 	errord2 = -1.0*velYManip_global; */
/* 	errord3 = -1.0*velThManip_global; */
/* 	errorInt1 += error1; */
/* 	errorInt2 += error2; */
/* 	errorInt3 += error3; */
/* 	xmdd = kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	ymdd = kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	thmdd = kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	th2dd = calculateJointAccelFromManipAccel2(xmdd, ymdd, thmdd); */
/* 	torque = controlManipAccel2(xmdd,ymdd,thmdd); */
/* //	reqCurrent = torque/KM2 + kp2curr*((thRH11global - th2Prev)+0.5*DT*DT*th2dd) + kd2curr*(DT*th2dd); */
/* 	break; */
/*     default: break; */
/*     } */
/*     th2Prev = thRH11global; */
/*     //Setting the control will check bounds */
/*     return reqCurrent; */
/* } */

/* //RH8, motor 3 */
/* double calculateControl3() { */
/*     //Initialize variables to 0 */
/*     double reqCurrent = 0; */
/*     //Required torque */
/*     double torque = 0; */
    
/*     double xmdd, ymdd, thmdd; */
/*     double xodd, yodd, thodd; */
/*     double errorX, errorY, errorTh; */
/*     static double th3Prev = 0.0; */

/*     double th3dd; */
    
/*     //Determine our control mode */
/*     switch(control_mode) { */
/*     case NO_CONTROL: */
/* 	//Don't do anything, use 0 current */
/* 	break; */
/*     case GO_HOME_JOINTS: */
/* 	//Go home */
/* 	//Calculate current error */
/* 	error3 = home3 - thRH8global; */
/* 	//Update integral error */
/* 	errorInt3 += error3*DT; */
/* 	//Use PI + gravComp for home control */
/* 	torque = KPH3*error3 + KIH3*errorInt3; */
/* 	reqCurrent = torque/KM3; */
/* 	break; */
/*     case TRAJ_IS_CURRENT: */
/* 	//Trajectories are actually current vals */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    reqCurrent = traj3[globalIndex]; */
/* 	} */
/* 	break; */
/*     case FEEDFORWARD_JOINTS: */
/* 	//Feedforward control */
/* 	torque = feedForward3(); */
/* 	reqCurrent = torque/KM3; */
/* 	break; */
/*     case PID_TRAJ_JOINTS: */
/* 	//PID Traj control */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error3 = traj3[globalIndex] - thRH8global; */
/* 	    errord3 = calculateTrajVel3() - velRH8global; */
/* 	    errorInt3 += error3*DT; */
/* 	    torque = kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	    reqCurrent = torque/KM3; */
/* 	} */
/* 	break; */
/*     case FF_PID_TRAJ_JOINTS: */
/* 	//Feedforward control with PID */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error3 = traj3[globalIndex] - thRH8global; */
/* 	    errord3 = calculateTrajVel3() - velRH8global; */
/* 	    errorInt3 += error3*DT; */
/* 	    torque = feedForward3() + kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	    reqCurrent = torque/KM3; */
/* 	} */
/* 	break; */
/*     case DYNAMIC_GRASP_TRAJ: */
/* 	//Dyanmic grasp follow a traj */
/* 	errorX = traj1[globalIndex] - xObjectGlobal; */
/* 	errorY = traj2[globalIndex] - yObjectGlobal; */
/* 	errorTh = traj3[globalIndex] - thObjectGlobal; */
/* 	//Check for minimum distance on circle */
/* 	if(fabs(errorTh) > fabs(2*M_PI - errorTh)) { */
/* 	    errorTh = 2.0*M_PI - errorTh; */
/* 	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) { */
/* 	    errorTh = 2.0*M_PI + errorTh; */
/* 	} */
/* 	//Add code here to go from errors in OBJECT accel to manip accel */
/* 	xodd = calculateTrajAccel1() + errorX*kp1; */
/* 	yodd = calculateTrajAccel2() + errorY*kp2; */
/* 	thodd = calculateTrajAccel3() + errorTh*kp3; */
/* 	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd); */
/* 	th3dd = calculateJointAccelFromManipAccel3(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel3(xmdd, ymdd, thmdd); */
/* //	reqCurrent = torque/KM3 + kp3curr*((thRH8global - th3Prev) + 0.5*DT*DT*th3dd) + kd3curr*(DT*th3dd); */
/* 	break; */
/*     case DYNAMIC_GRASP_POS: */
/* 	//Hold a pose in dynamic grasp */
/* 	errorX = objHomeX - xObjectGlobal; */
/* 	errorY = objHomeY - yObjectGlobal; */
/* 	errorTh = objHomeTh - thObjectGlobal; */
/* 	if(fabs(errorTh) > fabs(2*M_PI - errorTh)) { */
/* 	    errorTh = 2.0*M_PI - errorTh; */
/* 	} else if(fabs(errorTh) > fabs(2*M_PI + errorTh)) { */
/* 	    errorTh = 2.0*M_PI + errorTh; */
/* 	} */
/* 	xodd = errorX*kp1; */
/* 	yodd = errorY*kp2; */
/* 	thodd = errorTh*kp3; */
/* 	dynamicGraspControl(xodd, yodd, thodd, &xmdd, &ymdd, &thmdd); */
/* 	th3dd = calculateJointAccelFromManipAccel3(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel3(xmdd, ymdd, thmdd); */
/* //	reqCurrent = torque/KM3 + kp3curr*((thRH8global - th3Prev) + 0.5*DT*DT*th3dd) + kd3curr*(DT*th3dd); */
/* 	break; */
/*     case FF_PID_TRAJ_MANIP: */
/* 	if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	    error1 = traj1[globalIndex] - xManip_global; */
/* 	    error2 = traj2[globalIndex] - yManip_global; */
/* 	    error3 = traj3[globalIndex] - thManip_global; */
/* 	    if(fabs(error3) > fabs(2*M_PI - error3)) { */
/* 		error3 = 2.0*M_PI - error3; */
/* 	    } else if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 		error3 = 2.0*M_PI + error3; */
/* 	    } */
/* 	    errord1 = calculateTrajVel1() - velXManip_global; */
/* 	    errord2 = calculateTrajVel2() - velYManip_global; */
/* 	    errord3 = calculateTrajVel3() - velThManip_global; */
/* 	    errorInt1 += error1; */
/* 	    errorInt2 += error2; */
/* 	    errorInt3 += error3; */
/* 	    xmdd = calculateTrajAccel1() + kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	    ymdd = calculateTrajAccel2() + kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	    thmdd = calculateTrajAccel3() + kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	    th3dd = calculateJointAccelFromManipAccel3(xmdd,ymdd,thmdd); */
/* //	    desAccel3[globalIndex] = th3dd; */
/* 	    torque = controlManipAccel3(xmdd, ymdd, thmdd); */
/* //	    reqCurrent = torque/KM3 + kp3curr*((thRH8global - th3Prev) + 0.5*DT*DT*th3dd) + kd3curr*(DT*th3dd); */
/* 	} else reqCurrent = 0.0; */
/* 	break; */
/*     case PID_MANIP_POS: */
/* 	error1 = home1 - xManip_global; */
/* 	error2 = home2 - yManip_global; */
/* 	error3 = home3 - thManip_global; */
/* 	if(fabs(error3) > fabs(2*M_PI - error3)) { */
/* 	    error3 = 2.0*M_PI - error3; */
/* 	} else if(fabs(error3) > fabs(2*M_PI + error3)) { */
/* 	    error3 = 2.0*M_PI + error3; */
/* 	} */
/* 	errord1 = -1.0*velXManip_global; */
/* 	errord2 = -1.0*velYManip_global; */
/* 	errord3 = -1.0*velThManip_global; */
/* 	errorInt1 += error1; */
/* 	errorInt2 += error2; */
/* 	errorInt3 += error3; */
/* 	xmdd = kp1*error1 + kd1*errord1 + ki1*errorInt1; */
/* 	ymdd = kp2*error2 + kd2*errord2 + ki2*errorInt2; */
/* 	thmdd = kp3*error3 + kd3*errord3 + ki3*errorInt3; */
/* 	th3dd = calculateJointAccelFromManipAccel3(xmdd,ymdd,thmdd); */
/* 	torque = controlManipAccel3(xmdd,ymdd,thmdd); */
/* //	reqCurrent = torque/KM3 + kp3curr*((thRH8global - th3Prev) + 0.5*DT*DT*th3dd) + kd3curr*(DT*th3dd); */
/* 	break; */
/*     default: break; */
/*     } */
/*     th3Prev = thRH8global; */
/*     //Setting the control will check bounds */
/*     return reqCurrent; */
/* } */

/* double feedForward1() { */
/*     double s1, s2, s12, c1, c2, c12, th1dot, th2dot, th1ddot, th2ddot, th3ddot, torqueDes, torqueFriction; */
/*     double th1, th2; */

/*     if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	th1 = traj1[globalIndex]; */
/* 	th2 = traj2[globalIndex]; */
/* 	s1 = sin(th1); */
/* 	s2 = sin(th2); */
/* 	s12 = sin(th1+th2); */
/* 	c1 = cos(th1); */
/* 	c2 = cos(th2); */
/* 	c12 = cos(th1+th2); */
/* 	th1dot = calculateTrajVel1(); */
/* 	th2dot = calculateTrajVel2(); */
/* 	th1ddot = calculateTrajVel1(); */
/* 	th2ddot = calculateTrajVel2(); */
/* 	th3ddot = calculateTrajVel3(); */

/* 	torqueDes = -0.5*g*(L1*(m1+2.0*(m2 + m3 + mm2 + mm3))*c1 + \ */
/* 			    L2*(m2+2.0*(m3+mm3))*c12) - \ */
/* 	    L1*L2*(m2+2.0*(m3+mm3))*s2*th1dot*th2dot - \ */
/* 	    0.5*L1*L2*(m2+2.0*(m3+mm3))*s2*th2dot*th2dot + \ */
/* 	    (I1 + I2 +I3 + L2*L2*(m2*0.25+m3+mm3)+L1*L1*(m1*.25+m2+m3+mm2+mm3) + \ */
/* 	     J1+J2+J3 + L1*L2*(m2+2.0*(m3+mm3))*c2)*th1ddot + \ */
/* 	    (I2+I3+L2*L2*(m2*0.25+m3+mm3) + J2 + J3+ \ */
/* 	     0.5*L1*L2*(m2+2.0*(m3+mm3))*c2)*th2ddot + \ */
/* 	    (I3+J3)*th3ddot; */
/* 	torqueFriction = MUD1*th1dot; */
/* 	if (th1dot > 0.0) */
/* 	    torqueFriction += MUS1; */
/* 	else if(th1dot < 0.0) */
/* 	    torqueFriction -= MUS1; */
/* 	return torqueDes + torqueFriction; */
/*     } else */
/* 	return 0.0; */
/* } */

/* double feedForward2() { */
/*     double th1, th2, s12, s2, c12, c2, th1dot, th2dot, th1ddot, th2ddot, th3ddot, torqueDes, torqueFriction; */
/*     if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	th1 = traj1[globalIndex]; */
/* 	th2 = traj2[globalIndex]; */
/* 	s12 = sin(th1 + th2); */
/* 	c12 = cos(th1 + th2); */
/* 	s2 = sin(th2); */
/* 	c2 = cos(th2); */
/* 	th1dot = calculateTrajVel1(); */
/* 	th2dot = calculateTrajVel2(); */
/* 	th1ddot = calculateTrajAccel1(); */
/* 	th2ddot = calculateTrajAccel2(); */
/* 	th3ddot = calculateTrajAccel3(); */
/* 	torqueDes = 0.25*(-2.0*g*L2*(m2+2.0*(m3+mm3))*c12 + \ */
/* 			  2.0*L1*L2*(m2+2.0*(m3+mm3))*s2*th1dot*th1dot + \ */
/* 			  (4.0*I2 + 4.0*I3 + L2*L2*(m2+4.0*(m3+mm3)) + 4.0*(J2+J3) + \ */
/* 			   2.0*L1*L2*(m2+2.0*(m3+mm3))*c2)*th1ddot + \ */
/* 			  (4.0*I2+4.0*I3+L2*L2*(m2+4.0*(m3+mm3))+4.0*(J2+J3))*th2ddot + \ */
/* 			  4.0*(I3+J3)*th3ddot); */

/* 	torqueFriction = MUD2*th2dot; */
/* 	if(th2dot > 0.0) */
/* 	    torqueFriction += MUS2; */
/* 	else if(th2dot < 0.0) */
/* 	    torqueFriction -= MUS2; */
/* 	return torqueDes + torqueFriction; */
/*     } */
/*     else return 0.0; */
/* } */

/* double feedForward3() { */
/*     double torqueDes, th3dot, torqueFriction; */
/*     if(globalIndex >= 0 && globalIndex < num_pts && running) { */
/* 	torqueDes = (I3+J3)*(calculateTrajAccel1() + calculateTrajAccel2() + calculateTrajAccel3()); */
/* 	th3dot = calculateTrajVel3(); */
/* 	torqueFriction = MUD3*th3dot; */
/* 	if(th3dot > 0.0) */
/* 	    torqueFriction += MUS3; */
/* 	else if(th3dot < 0.0) */
/* 	    torqueFriction -= MUS3; */
/* 	return torqueDes + torqueFriction; */
/*     } else return 0.0; */
/* } */

/* double controlManipAccel1(double xmdd, double ymdd, double thmdd) { */
/*     static int printCount = 0; */
/*     static double oldTorqueDes = 0.0; */
/*     double th1, th2, th3, th1d, th2d, th3d; */
/*     double c1, c2, c12, s1, s12, s2, c122, s122, c1_2, s1_2; */
/*     double torqueFric, torqueDes; */
/*     th1 = thRH14global; */
/*     th2 = thRH11global; */
/*     th3 = thRH8global; */
/*     th1d = velRH14global; */
/*     th2d = velRH11global; */
/*     th3d = velRH8global; */
/*     printCount++; */

/*     if(fabs(th2) < 0.05) return oldTorqueDes; */
/*     torqueFric = th1d*MUD1; */
/*     if(th1d >= 0.0) */
/* 	torqueFric += MUS1; */
/*     else if(th1d < 0.0) */
/* 	torqueFric -= MUS1; */
/*     if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS) */
/*     	torqueFric = th1d*MUD1; */
/*     c1 = cos(th1); */
/*     c12 = cos(th1+th2); */
/*     c2 = cos(th2); */
/*     s1 = sin(th1); */
/*     s2 = sin(th2); */
/*     s12 = sin(th1+th2); */
/*     c122 = cos(th1+2.0*th2); */
/*     s122 = sin(th1+2.0*th2); */
/*     c1_2 = cos(th1-th2); */
/*     s1_2 = sin(th1-th2); */

/*     torqueDes = 0.25*(-2.0*g*(L1*(m1 + 2.0*m2 + 2.0*m3 + 2.0*mm2 + 2.0*mm3)*c1 + \ */
/* 			      L2*(m2+2.0*m3+2.0*mm3)*c12) - 4.0*L1*L2*(m2+2.0*m3+2.0*mm3)*s2*th1d*th2d - \ */
/* 		      2.0*L1*L2*(m2 + 2.0*m3 + 2.0*mm3)*s2*th2d*th2d + \ */
/* 		      1.0/(L1*L2)*(-2.0*L1*(2.0*I2 + L2*L2*(m3 + mm3) + 2.0*J2)*c1 + \ */
/* 				   L2*(-1.0*L1*L1*(m2+2.0*m3+2.0*mm3)*c1_2 + \ */
/* 				       (4.0*I1 + L1*L1*(m1+3.0*m2+2.0*m3+4.0*mm2+2.0*mm3) + \ */
/* 					4.0*J1)*c12 + L1*L2*(m2 + 2.0*m3 + 2.0*mm3)*c122)) * \ */
/* 		      1.0/s2*(L1*c1*th1d*th1d + L2*c1*c2*(th1d + th2d)*(th1d + th2d) - \ */
/* 			      L2*s1*s2*(th1d+th2d)*(th1d+th2d) - xmdd) + \ */
/* 		      1.0/(L1*L2*s2)*(-2.0*L1*(2.0*I2 + L2*L2*(m3+mm3) + 2.0*J2)*s1 + \ */
/* 				      L2*(-1.0*L1*L1*(m2+2.0*(m3+mm3))*s1_2 + \ */
/* 					  (4.0*I1 + L1*L1*(m1 + 3.0*m2+2.0*m3+4.0*mm2+2.0*mm3) + 4.0*J1)*s12 + \ */
/* 					  L1*L2*(m2+2.0*m3+2.0*mm3)*s122))* \ */
/* 		      (L1*s1*th1d*th1d + L2*c2*s1*(th1d+th2d)*(th1d+th2d) +	\ */
/* 		       L2*c1*s2*(th1d+th2d)*(th1d+th2d) - ymdd) + 4.0*(I3+J3)*thmdd); */
/*     torqueDes += torqueFric; */
/* //    if(printCount >= 1500) { */
/* //    	printf("th1: %f, th2: %f, th3: %f\n", th1, th2, th3); */
/* //    	printf("th1d: %f, th2d: %f, th3d: %f\n", th1d, th2d, th3d); */
/* //    	printf("staticFric: %f, torqueDes: %f\n", staticFric, torqueDes); */
/* //    	//c1, c2, c12, s1, s12, s2, c122, s122, c1_2, s1_2 */
/* //    	printf("c1: %f, c2: %f, c12: %f\n", c1, c2, c12); */
/* //    	printf("s1: %f, s12: %f, s2: %f\n", s1, s12, s2); */
/* //    	printf("c122: %f, s122: %f, c1_2: %f, s1_2: %f\n",c122, s122, c1_2, s1_2); */
/* //    	printCount = 0; */
/* //    } */
/*     oldTorqueDes = torqueDes; */
/*     return torqueDes; */
/* } */
/* double controlManipAccel2(double xmdd, double ymdd, double thmdd) { */
/*     double th1, th2, th1d, th2d; */
/*     double c1, c2, c12, c122, s1, s2, s122; */
/*     static double oldTorqueDes = 0.0; */
/*     th1 = thRH14global; */
/*     th2 = thRH11global; */
/*     th1d = velRH14global; */
/*     th2d = velRH11global; */
/*     double torqueFric, torqueDes; */
/*     torqueFric = th2d*MUD2; */
/*     if(fabs(th2) < 0.05) return oldTorqueDes; */
/*     if(th2d >= 0.0) */
/* 	torqueFric += MUS2; */
/*     else if(th2d < 0.0) */
/* 	torqueFric -= MUS2; */
/*     if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS) */
/*         	torqueFric = th2d*MUD2; */
/*     c1 = cos(th1); */
/*     c2 = cos(th2); */
/*     c12 = cos(th1+th2); */
/*     s1 = sin(th1); */
/*     s2 = sin(th2); */
/*     c122 = cos(th1+2.0*th2); */
/*     s122 = sin(th1+2.0*th2); */
/*     torqueDes = 1.0/(4.0*L2)*(-2.0*g*L2*L2*(m2+2.0*m3+2.0*mm3)*c12 - \ */
/* 			      (4.0*I2-L2*L2*m2 + 4.0*J2)*(L1+L2*c2)*th1d*th1d/s2 + \ */
/* 			      L2*th2d*(-1.0*(4.0*I2 - L2*L2*m2 + 4.0*J2)*c2/s2*(2.0*th1d + th2d)) + \ */
/* 			      1.0/s2*((2.0*(2.0*I2 + L2*L2*(m3+mm3) + 2.0*J2)*c1 - \ */
/* 				       L2*L2*(m2+2.0*m3+2.0*mm3)*c122)*xmdd + \ */
/* 				      (2.0*(2.0*I2 + L2*L2*(m3+mm3) + 2.0*J2)*s1 - \ */
/* 				       L2*L2*(m2 +2.0*m3+2.0*mm3)*s122)*ymdd) + \ */
/* 			      4.0*L2*(I3+J3)*thmdd); */
/*     torqueDes += torqueFric; */
/* //    printf("tau2: %f\n",torqueDes); */
/*     oldTorqueDes = torqueDes; */
/*     return torqueDes; */
/* } */

/* double controlManipAccel3(double xmdd, double ymdd, double thmdd) { */
/*     double th3, torqueFric, th3d; */
/*     th3 = thRH8global; th3d = velRH8global; */
/*     torqueFric = th3d*MUD3; */
/*     if(th3d >= 0.0) */
/* 	torqueFric += MUS3; */
/*     else if(th3d < 0.0) */
/* 	torqueFric -= MUS3; */
/*     if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS) */
/*         	torqueFric = th3d*MUD3; */
/* //    printf("tau3: %f\n",staticFric + MUD3*th3d + (I3 + J3)*thmdd); */

/*     return torqueFric + (I3 + J3)*thmdd; */
/* } */

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

//Takes a set of desired object accelerations (ficticious controls)
//Then calculates required manipulator accelerations (based on kinematic equations)
//These controls are still fictitous, but they are translated by later functions to motor commands
void dynamicGraspControl(double xodd, double yodd, double thodd, double *xmdd, double *ymdd, double *thmdd) {
    static int printTimer = 0;
    double cm, sm, thmd, thm;
    thm = thManip_global;
    thmd = velThManip_global;
    cm = cos(thm);
    sm = sin(thm);
	
    *thmdd = thodd;
    *xmdd = ((contactPoint1-lm+lo)*cm - (wm + wo)*sm)*thmd*thmd + xodd + \
	((wm+wo)*cm + (contactPoint1 - lm + lo)*sm)*thodd;
    *ymdd = ((wm+wo)*cm + (contactPoint1 - lm + lo)*sm)*thmd*thmd + yodd + \
	((lm - contactPoint1 - lo)*cm + (wm + wo)*sm)*thodd;

    printTimer++;
//    if(printTimer >= 10000 && control_mode != NO_CONTROL) {
////    	printf("Curr1: %f, curr2: %f, curr3: %f\n", desCur1, desCur2, desCur3);
////    	printf("th1: %f, th2: %f, th3: %f\n",thRH14global, thRH11global,thRH8global);
//    	printf("xm: %f, ym: %f, thm: :%f\n", xManip_global, yManip_global, thManip_global);
////    	printf("th1d: %f, th2d: %f, th3d: %f\n", velRH14global, velRH11global, velRH8global);
//    	printf("xObj: %f, yObj: %f, thObj: %f\n", xObjectGlobal, yObjectGlobal, thObjectGlobal);
//    	printf("velXObj: %f, velYObj: %f, velthObj: %f\n", velXObjectGlobal, velYObjectGlobal, velThObjectGlobal);
//    	printf("Contact1: %f, contact2: %f\n", contactPoint1, contactPoint2);
//    	printf("xodd: %f, yodd: %f, thodd: %f\n", xodd, yodd, thodd);
//    	printf("xmdd: %f, ymdd: %f, thmdd: %f\n\n", *xmdd, *ymdd, *thmdd);
//    	printTimer = 0;
//    }
}

void arcLengthContactPoints() {
    double xc1, xm, thm, yc1, ym, th1, th2, th3;
    th1 = thRH14global;
    th2 = thRH11global;
    th3 = thRH8global;
    xm = xManip_global;
    ym = yManip_global;
    thm = thManip_global;
//Calculate xc1 in terms of object coordinates

    //Now, solve for contact point 1
    //Check if near singularity with cos, if so, use Y data points
    if(fabs(fabs(thm) - M_PI/2.0) > 0.1 && fabs(fabs(thm) - 1.5*M_PI) > 0.1)  {
	xc1 = xObjectGlobal - lo*cos(thObjectGlobal) + wo*sin(thObjectGlobal);
	contactPoint1 = (xc1 + lm*cos(thm) + wm*sin(thm) - xm)/cos(thm);
    } else {
	yc1 = yObjectGlobal - wo*cos(thObjectGlobal) - lo*sin(thObjectGlobal);
	contactPoint1 = (yc1 - ym - wm*cos(thm) + lm*sin(thm))/sin(thm);
    }
    contactPoint2 = contactPoint1 + 2.0*lo;
    //print just to check if this makes sense
//    if(newCameraData) {
//    printf("xM: %f, yM: %f, thm: %f\n",xm,ym,thm);
//    printf("xo: %f, yo: %f, tho: %f\n",xObjectGlobal, yObjectGlobal, thObjectGlobal);
//    printf("s1: %f, s2: %f\n", contactPoint1, contactPoint2);
//    printf("xRH14: %f, yRH14: %f, thRH14: %f\n", xGlobal[0], yGlobal[0], th1);
//    printf("xRH11: %f, yRH11: %f, thRH11: %f\n", xGlobal[1], yGlobal[1], th2);
//    printf("xRH8: %f, yRH8: %f, thRH8: %f\n", xGlobal[2], yGlobal[2], th3);
//    printf("xMark1: %f, yMark1: %f\n",xGlobal[3], yGlobal[3]);
//    printf("xMark2: %f, yMark2: %f\n",xGlobal[4], yGlobal[4]);
//    printf("Area1: %d, Area2: %d, Area3: %d, Area4: %d, Area5: %d\n", areaGlobal[0], \
//    		areaGlobal[1], areaGlobal[2], areaGlobal[3], areaGlobal[4]);
//    }
//    else { printf("No new cam data");}
}

/* double calculateJointAccelFromManipAccel1(double xmdd, double ymdd, double thmdd) { */
/*     double th1, th2, th1d, th2d; */
/*     double s2, c2, c12, s12; */
/*     double th1dd; */
/*     static double th1ddOld = 0.0; */
/*     th1 = thRH14global; */
/*     th2 = thRH11global; */
/*     th1d = velRH14global; */
/*     th2d = velRH11global; */
/*     if(fabs(th2) < 0.01) return th1ddOld; */

/*     s2 = sin(th2); c2 = cos(th2); */
/*     c12 = cos(th1+th2); s12 = sin(th1+th2); */

/*     th1dd = 1.0/L1*(1.0/s2*(L1*c2*th1d*th1d + L2*(th1d+th2d)*(th1d+th2d) - c12*xmdd - s12*ymdd)); */
/*     th1ddOld = th1dd; */
/*     return th1dd; */
/* } */

/* double calculateJointAccelFromManipAccel2(double xmdd, double ymdd, double thmdd) { */
/*     double th1, th2, th1d, th2d; */
/*     double s2, c2, c1, c12, s1, s12; */
/*     double th2dd; */
/*     static double th2ddOld = 0.0; */
/*     th1 = thRH14global; */
/*     th2 = thRH11global; */
/*     th1d = velRH14global; */
/*     th2d = velRH11global; */
/*     if(fabs(th2) < 0.01) return th2ddOld; */

/*     s2 = sin(th2); c2 = cos(th2); */
/*     c1 = cos(th1); c12 = cos(th1+th2); */
/*     s1 = sin(th1); s12 = sin(th1+th2); */

/*     th2dd = 1.0/(L1*L2*s2)*(-1.0*(L1*L1+L2*L2+2*L1*L2*c2)*th1d*th1d - \ */
/* 			    2.0*L2*(L2 + L1*c2)*th1d*th2d - \ */
/* 			    L2*(L2+L1*c2)*th2d*th2d + \ */
/* 			    (L1*c1+L2*c12)*xmdd + \ */
/* 			    (L1*s1+L2*s12)*ymdd); */
/*     th2ddOld = th2dd; */
/*     return th2dd; */
/* } */

/* double calculateJointAccelFromManipAccel3(double xmdd, double ymdd, double thmdd) { */
/*     double th1, th2, th1d, th2d; */
/*     double s2, c2, c1, s1; */
/*     th1 = thRH14global; */
/*     th2 = thRH11global; */
/*     th1d = velRH14global; */
/*     th2d = velRH11global; */
/*     if(fabs(th2) < 0.01) return thmdd; */
/*     s2 = sin(th2); s1 = sin(th1); */
/*     c2 = cos(th2); c1 = cos(th1); */
    
/*     return 1.0/(sin(th2)*L2)*(L1*th1d*th1d + L2*c2*(th1d+th2d)* \ */
/* 			      (th1d+th2d) - c1*xmdd - s1*ymdd) + \ */
/* 	thmdd; */
/* } */

void robotTorques(double *torqueDes1, double *torqueDes2, double *torqueDes3, \
		  double th1ddot, double th2ddot, double th3ddot, \
		  double th1dot, double th2dot, double th3dot, \
		  double th1, double th2, double th3) {
    double s1, s2, s12, c1, c2, c12, torqueFric1, torqueFric2, torqueFric3, staticFric;
    s1 = sin(th1); c1 = cos(th1);
    s2 = sin(th2); c2 = cos(th2);
    s12 = sin(th1+th2);
    c12 = cos(th1+th2);
    *torqueDes1 = -0.5*g*(L1*(m1+2.0*(m2 + m3 + mm2 + mm3))*c1 +	\
			  L2*(m2+2.0*(m3+mm3))*c12) -			\
	L1*L2*(m2+2.0*(m3+mm3))*s2*th1dot*th2dot -			\
	0.5*L1*L2*(m2+2.0*(m3+mm3))*s2*th2dot*th2dot +			\
	(I1 + I2 +I3 + L2*L2*(m2*0.25+m3+mm3)+L1*L1*(m1*.25+m2+m3+mm2+mm3) + \
	 J1+J2+J3 + L1*L2*(m2+2.0*(m3+mm3))*c2)*th1ddot +		\
	(I2+I3+L2*L2*(m2*0.25+m3+mm3) + J2 + J3+			\
	 0.5*L1*L2*(m2+2.0*(m3+mm3))*c2)*th2ddot +			\
	(I3+J3)*th3ddot;
    if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS  || control_mode == GO_HOME_JOINTS)
    	staticFric = 0.0;
    else
    	staticFric = MUS1;
    torqueFric1 = MUD1*th1dot;
    if (th1dot > 0.0)
	torqueFric1 += staticFric;
    else if(th1dot < 0.0)
	torqueFric1 -= staticFric;
    *torqueDes1 += torqueFric1;

    //th2
    *torqueDes2 = 0.25*(-2.0*g*L2*(m2+2.0*(m3+mm3))*c12 +		\
			2.0*L1*L2*(m2+2.0*(m3+mm3))*s2*th1dot*th1dot +	\
			(4.0*I2 + 4.0*I3 + L2*L2*(m2+4.0*(m3+mm3)) + 4.0*(J2+J3) + \
			 2.0*L1*L2*(m2+2.0*(m3+mm3))*c2)*th1ddot +	\
			(4.0*I2+4.0*I3+L2*L2*(m2+4.0*(m3+mm3))+4.0*(J2+J3))*th2ddot + \
			4.0*(I3+J3)*th3ddot);

    if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS || control_mode == GO_HOME_JOINTS)
        	staticFric = 0.0;
    else
        	staticFric = MUS2;
    torqueFric2 = MUD2*th2dot;
    if(th2dot > 0.0)
	torqueFric2 += staticFric;
    else if(th2dot < 0.0)
	torqueFric2 -= staticFric;
    *torqueDes2 += torqueFric2;

    //th3
    if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS || control_mode == GO_HOME_JOINTS)
    	staticFric = 0.0;
    else
    	staticFric = MUS3;//MUS3;
    torqueFric3 = MUD3*th3dot;

    if(th3dot > 0.0)
	torqueFric3 += staticFric;
    else if(th3dot < 0.0)
	torqueFric3 -= staticFric;
    *torqueDes3 = torqueFric3 + (J3+I3)*(th1ddot + th2ddot + th3ddot);
}

void calculateJointAccelFromManipAccel(double xmdd, double ymdd, double thmdd, \
				       double *th1dd, double *th2dd, double *th3dd) {
    double th1, th2, th1d, th2d;
    double s1, s2, c1, c2, c12, s12;
    static double th1ddOld = 0.0;
    static double th2ddOld = 0.0;
    static double th3ddOld = 0.0;

    th1 = thRH14global;
    th2 = thRH11global;
    th1d = velRH14global;
    th2d = velRH11global;
    //Can't do inverse dynamics near singularity of jacobian
    //Jacobian is singular if th2 = 0
    if(fabs(th2) < 0.005) {
    //	printf("Singular!\n");
	*th1dd = th1ddOld;
	*th2dd = th2ddOld;
	*th3dd = th3ddOld;
    } else {
	s1 = sin(th1);
	s2 = sin(th2);
	c1 = cos(th1);
	c2 = cos(th2);
	c12 = cos(th1 + th2);
	s12 = sin(th1 + th2);

	*th1dd = 1.0/L1*(1.0/s2*(L1*c2*th1d*th1d + L2*(th1d+th2d)*(th1d+th2d) - c12*xmdd - s12*ymdd));
	*th2dd = 1.0/(L1*L2*s2)*(-1.0*(L1*L1+L2*L2+2*L1*L2*c2)*th1d*th1d - \
				 2.0*L2*(L2 + L1*c2)*th1d*th2d -	\
				 L2*(L2+L1*c2)*th2d*th2d +  \
				 (L1*c1+L2*c12)*xmdd +	\
				 (L1*s1+L2*s12)*ymdd);
	*th3dd = 1.0/(sin(th2)*L2)*(L1*th1d*th1d + L2*c2*(th1d+th2d)*	\
				    (th1d+th2d) - c1*xmdd - s1*ymdd) +	\
	    thmdd;
	th1ddOld = *th1dd;
	th2ddOld = *th2dd;
	th3ddOld = *th3dd;
    }
}
