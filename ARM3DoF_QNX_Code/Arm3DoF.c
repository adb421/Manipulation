/*
 * Arm3DoF.c
 *
 *  Created on: Oct 16, 2012
 *      Author: Adam
 */

#include "Arm3DoF.h"

void initialize_variables() {
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
    end_program = 0;
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

void flip_pin(uintptr_t iobase) {
	static short pin = 0;
	if(pin) {
		//Pin was high, set it low
		DIO_word = DIO_word & 0xFFBF;
		pin = 0;
	} else {
		//Pin was low, set it high
		DIO_word = DIO_word | 0x0840;
		pin = 1;
	}
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
	total_count = (int)((thObjectGlobal-calculateJointTwoCamera() - calculateJointOneCamera())/M_PI*51200.0);
//	total_count = (int)((-calculateJointTwoCamera() - calculateJointOneCamera())/M_PI*51200.0);
//		total_count = 0;
    }
    //total_count = read_encoder_1(iobase);
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
    	DACVal = 65530;
    } else if((-1.0*desCurrent) >= (MAX_CURRENT_RH14*1.0 - MAX_CURRENT_RH14/32767.5)) {
    	DACVal = 5;
    } else {
    	DACVal = (uint16_t)((desCurrent/(MAX_CURRENT_RH14*1.0) + 1.0)*32767.5);
    }
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
    free(traj4);
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
    free(desXAccel);
    free(desYAccel);
    free(desThAccel);
    free(desAccel1);
    free(desAccel2);
    free(desAccel3);
    free(uffX);
    free(uffY);
    free(uffTh);
    free(KT11);
    free(KT12);
    free(KT13);
    free(KT14);
    free(KT15);
    free(KT16);
    free(KT17);
    free(KT18);
    free(KT21);
    free(KT22);
    free(KT23);
    free(KT24);
    free(KT25);
    free(KT26);
    free(KT27);
    free(KT28);
    free(KT31);
    free(KT32);
    free(KT33);
    free(KT34);
    free(KT35);
    free(KT36);
    free(KT37);
    free(KT38);
    //Ensure control mode is 0 and no control
    control_mode = NO_CONTROL;
    running = 0;
    //Reset errors
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
    printf("Reset\n");
}

//Calculate control for all three motors
void calculateControl(double *reqCurrentRH14, double *reqCurrentRH11, double *reqCurrentRH8){
	static int resetTimer = 0;
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

    double errorXm, errorXmDot;
    double errorYm, errorYmDot;
    double errorThm, errorThmDot;
    double errorXo, errorXoDot;
    double errorYo, errorYoDot;
    double errorTho, errorThoDot;
    double error_inner_d_RH8, error_inner_RH8;
    double error_inner_d_RH11, error_inner_RH11;
    double error_inner_d_RH14, error_inner_RH14;
    
    static short reset_inner_loop = 1;

    static double virtualRH8 = 0.0;
    static double virtualRH11 = 0.0;
    static double virtualRH14 = 0.0;
    static double virtualVelRH8 = 0.0;
    static double virtualVelRH11 = 0.0;
    static double virtualVelRH14 = 0.0;

    //Switch based on control_mode
    switch(control_mode) {
    case NO_CONTROL:
	//Don't do anything, 0 current
//	th1Prev = thRH14global;
//	th2Prev = thRH11global;
//	th3Prev = thRH8global;
    reset_inner_loop = 1;
    if(globalIndex >= 0 && globalIndex < num_pts && running) {
    	desXAccel[globalIndex] = 0.0;
		desYAccel[globalIndex] = 0.0;
		desThAccel[globalIndex] = 0.0;
		desAccel1[globalIndex] = 0.0;
		desAccel2[globalIndex] = 0.0;
		desAccel3[globalIndex] = 0.0;
    }
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
	reset_inner_loop = 1;
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
	reset_inner_loop = 1;
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
	    desXAccel[globalIndex] = xmdd;
	    desYAccel[globalIndex] = ymdd;
	    desThAccel[globalIndex] = thmdd;
	    desAccel1[globalIndex] = th1dd;
	    desAccel2[globalIndex] = th2dd;
	    desAccel3[globalIndex] = th3dd;
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
	    desXAccel[globalIndex] = xmdd;
	    desYAccel[globalIndex] = ymdd;
	    desThAccel[globalIndex] = thmdd;
	    desAccel1[globalIndex] = th1dd;
	    desAccel2[globalIndex] = th2dd;
	    desAccel3[globalIndex] = th3dd;
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
	xmdd = 285.0*error1 + 90.0*errord1 + 0.0003*errorInt1;
	ymdd = 285.0*error2 + 90.0*errord2 + 0.0003*errorInt2;
	thmdd = 900.0*error3 + 100.0*errord3 + 0.0004*errorInt3;
	calculateJointAccelFromManipAccel(xmdd, ymdd, thmdd, &th1dd, &th2dd, &th3dd);
	reset_inner_loop = 1;
	break;
    case ONE_POINT_ROLL_BALANCE:
	//LQR errors
	errorThm = -thManip_global;
	errorThmDot = -velThManip_global;
	errorXo = X_OBJ_ROLL_GOAL - xObjectGlobal;
	errorXoDot = -velXObjectGlobal;
	errorYo = Y_OBJ_ROLL_GOAL - yObjectGlobal;
	errorYoDot = -velYObjectGlobal;
	errorTho = TH_OBJ_ROLL_GOAL - thObjectGlobal;
	errorThoDot = -velThObjectGlobal;
	xmdd = K_lqr[0]*errorThm + K_lqr[1]*errorThmDot + K_lqr[2]*errorXo + K_lqr[3]*errorXoDot + \
			K_lqr[4]*errorYo + K_lqr[5]*errorYoDot + K_lqr[6]*errorTho + K_lqr[7]*errorThoDot;
	ymdd = K_lqr[8]*errorThm + K_lqr[9]*errorThmDot + K_lqr[10]*errorXo + K_lqr[11]*errorXoDot + \
			K_lqr[12]*errorYo + K_lqr[13]*errorYoDot + K_lqr[14]*errorTho + K_lqr[15]*errorThoDot;
	thmdd = K_lqr[16]*errorThm + K_lqr[17]*errorThmDot + K_lqr[18]*errorXo + K_lqr[19]*errorXoDot + \
			K_lqr[20]*errorYo + K_lqr[21]*errorYoDot + K_lqr[22]*errorTho + K_lqr[23]*errorThoDot;
	if(ymdd < -0.8*g) {
		//printf("ClippingControl");
		ymdd = -0.8*g;
	}
	calculateJointAccelFromManipAccel(xmdd,ymdd,thmdd,&th1dd,&th2dd,&th3dd);
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
	    desXAccel[globalIndex] = xmdd;
	    desYAccel[globalIndex] = ymdd;
	    desThAccel[globalIndex] = thmdd;
	    desAccel1[globalIndex] = th1dd;
	    desAccel2[globalIndex] = th2dd;
	    desAccel3[globalIndex] = th3dd;
	}
	break;
    case NEW_ONE_POINT_ROLL_BALANCE:
	//LQR errors
	errorXm = X_MANIP_ROLL_GOAL - xManip_global;
	errorXmDot = -velXManip_global;
	errorYm = Y_MANIP_ROLL_GOAL - yManip_global;
	errorYmDot = -velYManip_global;
	errorThm = -thManip_global;
	errorThmDot = -velThManip_global;
	errorTho = TH_OBJ_ROLL_GOAL - thObjectGlobal;
	errorThoDot = -velThObjectGlobal;
	xmdd = K_lqr[0]*errorXm + K_lqr[1]*errorXmDot + K_lqr[2]*errorYm + K_lqr[3]*errorYmDot + \
			K_lqr[4]*errorThm + K_lqr[5]*errorThmDot + K_lqr[6]*errorTho + K_lqr[7]*errorThoDot;
	ymdd = K_lqr[8]*errorXm + K_lqr[9]*errorXmDot + K_lqr[10]*errorYm + K_lqr[11]*errorYmDot + \
			K_lqr[12]*errorThm + K_lqr[13]*errorThmDot + K_lqr[14]*errorTho + K_lqr[15]*errorThoDot;
	thmdd = K_lqr[16]*errorXm + K_lqr[17]*errorXmDot + K_lqr[18]*errorYm + K_lqr[19]*errorYmDot + \
			K_lqr[20]*errorThm + K_lqr[21]*errorThmDot + K_lqr[22]*errorTho + K_lqr[23]*errorThoDot;
	if(ymdd < -0.8*g) {
	    //printf("Clipping control\n");
	    ymdd = -0.8*g;
	}
	calculateJointAccelFromManipAccel(xmdd, ymdd, thmdd, &th1dd, &th2dd, &th3dd);
	if(globalIndex >= 0 && globalIndex < num_pts && running) {
	    desXAccel[globalIndex] = xmdd;
	    desYAccel[globalIndex] = ymdd;
	    desThAccel[globalIndex] = thmdd;
	    desAccel1[globalIndex] = th1dd;
	    desAccel2[globalIndex] = th2dd;
	    desAccel3[globalIndex] = th3dd;
	}
	break;
    case ONE_POINT_ROLL_TRAJ:
    if(globalIndex >= 0 && globalIndex < num_pts && running) {
    	//LQR errors
    	errorThm = traj1[globalIndex] - thManip_global;
    	errorThmDot = calculateTrajVel(traj1) - velThManip_global;
    	errorXo = traj2[globalIndex] - xObjectGlobal;
    	errorXoDot = calculateTrajVel(traj2) - velXObjectGlobal;
    	errorYo = traj3[globalIndex] - yObjectGlobal;
    	errorYoDot = calculateTrajVel(traj3) - velYObjectGlobal;
    	errorTho = traj4[globalIndex] - thObjectGlobal;
    	errorThoDot = calculateTrajVel(traj4) - velThObjectGlobal;
    	xmdd = KT11[globalIndex]*errorThm + KT12[globalIndex]*errorThmDot + KT13[globalIndex]*errorXo + \
    	       KT14[globalIndex]*errorXoDot + KT15[globalIndex]*errorYo + KT16[globalIndex]*errorYoDot + \
    	       KT17[globalIndex]*errorTho + KT18[globalIndex]*errorThoDot + uffX[globalIndex];
    	ymdd = KT21[globalIndex]*errorThm + KT22[globalIndex]*errorThmDot + KT23[globalIndex]*errorXo + \
    			KT24[globalIndex]*errorXoDot + KT25[globalIndex]*errorYo + KT26[globalIndex]*errorYoDot + \
    			KT27[globalIndex]*errorTho + KT28[globalIndex]*errorThoDot + uffY[globalIndex];
    	thmdd = KT31[globalIndex]*errorThm + KT32[globalIndex]*errorThmDot + KT33[globalIndex]*errorXo + \
    			KT34[globalIndex]*errorXoDot + KT35[globalIndex]*errorYo + KT36[globalIndex]*errorYoDot + \
    			KT37[globalIndex]*errorTho + KT38[globalIndex]*errorThoDot + uffTh[globalIndex];
    	if(ymdd < -0.8*g) {
    			//printf("ClippingControl");
    		ymdd = -0.8*g;
    	}
    	calculateJointAccelFromManipAccel(xmdd,ymdd,thmdd,&th1dd,&th2dd,&th3dd);
    	desXAccel[globalIndex] = xmdd;
    	desYAccel[globalIndex] = ymdd;
    	desThAccel[globalIndex] = thmdd;
    	desAccel1[globalIndex] = th1dd;
    	desAccel2[globalIndex] = th2dd;
    	desAccel3[globalIndex] = th3dd;
    }
    break;
    default:
	return;
	break;
    }
    //if(!reset_inner_loop && resetTimer <= 500) {
    if(!reset_inner_loop) {
    	//Update virtual trajectory and calculate errors
    	virtualRH14 += virtualVelRH14*DT + th1dd/2.0*DT*DT;
    	virtualVelRH14 += th1dd*DT;
    	error_inner_d_RH14 = virtualVelRH14 - velRH14global;
    	error_inner_RH14 = virtualRH14 - thRH14global;

    	virtualRH11 += virtualVelRH11*DT + th2dd/2.0*DT*DT;
    	virtualVelRH11 += th2dd*DT;
    	error_inner_d_RH11 = virtualVelRH11 - velRH11global;
    	error_inner_RH11 = virtualRH11 - thRH11global;

    	virtualRH8 += virtualVelRH8*DT + th3dd/2.0*DT*DT;
    	virtualVelRH8 += th3dd*DT;
    	error_inner_d_RH8 = virtualVelRH8 - velRH8global;
    	error_inner_RH8 = virtualRH8 - thRH8global;
    	//resetTimer++;
    } else {
    	virtualRH8 = thRH8global;
    	virtualRH11 = thRH11global;
    	virtualRH14 = thRH14global;
    	virtualVelRH8 = velRH8global;
    	virtualVelRH11 = velRH11global;
    	virtualVelRH14 = velRH14global;
    	reset_inner_loop = 0;
    	error_inner_d_RH14 = 0.0; error_inner_RH14 = 0.0;
    	error_inner_d_RH11 = 0.0; error_inner_RH11 = 0.0;
    	error_inner_d_RH8 = 0.0; error_inner_RH8 = 0.0;
    	resetTimer = 0;
    }

    robotTorques(&torqueDes1, &torqueDes2, &torqueDes3,		 \
		 th1dd, th2dd, th3dd,				 \
		 velRH14global, velRH11global, velRH8global,	 \
		 thRH14global, thRH11global, thRH8global);

    *reqCurrentRH14 = torqueDes1/KM1 + INNER_K1_14 * error_inner_RH14 + INNER_K2_14*error_inner_d_RH14;
    *reqCurrentRH11 = torqueDes2/KM2 + INNER_K1_11 * error_inner_RH11 + INNER_K2_11*error_inner_d_RH11;
    *reqCurrentRH8  = torqueDes3/KM3 + INNER_K1_8 * error_inner_RH8 + INNER_K2_8*error_inner_d_RH8;
}

double calculateTrajVel1(void) {
    if(globalIndex < 1 || globalIndex >= num_pts-1) return 0.0;
    else {
	return (traj1[globalIndex+1] - traj1[globalIndex-1])/(2.0*DT);
    }
}

double calculateTrajVel2(void) {
    if(globalIndex < 1 || globalIndex >= num_pts-1) return 0.0;
    else {
	return (traj2[globalIndex+1] - traj2[globalIndex-1])/(2.0*DT);
    }
}

double calculateTrajVel3(void) {
    if(globalIndex < 1 || globalIndex >= num_pts-1) return 0.0;
    else {
	return (traj3[globalIndex+1] - traj3[globalIndex-1])/(2.0*DT);
    }
}

double calculateTrajVel(double *traj) {
	if(globalIndex < 1 || globalIndex >= num_pts - 1) return 0.0;
	else {
		return (traj3[globalIndex+1] - traj3[globalIndex-1])/(2.0*DT);
	}
}

double calculateTrajAccel1(void) {
    if(globalIndex < 2 || globalIndex >= num_pts - 2) return 0.0;
    else {
	return ((traj1[globalIndex + 2] + traj1[globalIndex - 2] - 2.0*traj1[globalIndex])/(4.0*DT*DT) );
    }
}

double calculateTrajAccel2(void) {
    if(globalIndex < 2 || globalIndex >= num_pts - 2) return 0.0;
    else {
	return ((traj2[globalIndex + 2] + traj2[globalIndex - 2] - 2.0*traj2[globalIndex])/(4.0*DT*DT) );
    }
}

double calculateTrajAccel3(void) {
    if(globalIndex < 2 || globalIndex >= num_pts - 2) return 0.0;
    else {
	return ((traj3[globalIndex + 2] + traj3[globalIndex - 2] - 2.0*traj3[globalIndex])/(4.0*DT*DT) );
    }
}

//Takes a set of desired object accelerations (ficticious controls)
//Then calculates required manipulator accelerations (based on kinematic equations)
//These controls are still fictitous, but they are translated by later functions to motor commands
void dynamicGraspControl(double xodd, double yodd, double thodd, double *xmdd, double *ymdd, double *thmdd) {
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
}

//This calculates tau = M qdd + C + G + Ff
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
    if(control_mode == PID_MANIP_POS || control_mode == DYNAMIC_GRASP_POS || control_mode == GO_HOME_JOINTS \
    		|| control_mode == ONE_POINT_ROLL_BALANCE || control_mode == ONE_POINT_ROLL_TRAJ)
    	staticFric = 0.0;
    else
    	staticFric = MUS3;
    torqueFric3 = MUD3*th3dot;

    if(th3dot > 0.0)
	torqueFric3 += staticFric;
    else if(th3dot < 0.0)
	torqueFric3 -= staticFric;
    *torqueDes3 = torqueFric3 + (J3+I3)*(th1ddot + th2ddot + th3ddot);
}

//This calculates qdd = J^-1 (xdd - Jdot J^-1 xd)
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
				    (th1d+th2d) - c1*xmdd - s1*ymdd) +	thmdd;
	th1ddOld = *th1dd;
	th2ddOld = *th2dd;
	th3ddOld = *th3dd;
    }
}

int limitsExceeded() {
    if(fabs(thRH11global) > 3.0) { //May want to add in velocities
	printf("Joint 2 angle limit exceeded\n");
	return 1;
    }
    if(fabs(thRH14global) > 2.5) { //Velocities?
	printf("Joint 1 angle limit exceeded\n");
	return 1;
    }
    if(xManip_global > 0.12) {
	printf("X position past minimum\n");
	return 1;
    }
    return 0;
}
