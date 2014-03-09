/*
 * process_commands.c
 *
 *  Created on: Oct 30, 2012
 *      Author: Adam
 */

#include "process_commands.h"

void process_cmd() {
    int cmd;
    readInt(&cmd);
    waiting_for_cmd = 0;
    switch(cmd) {
    case 1:
	//Get number of points for trajectory from PC and init arrays
	procCmd1();
	break;
    case 2:
	//Receive joint trajectory from PC
	procCmd2();
	break;
    case 3:
	//Send back data from the previous run to PC and do a "soft reset"
	procCmd3();
	break;
    case 4:
	//Go to home position
	procCmd4();
	break;
    case 5:
	//execute trajectory
	procCmd5();
	break;
    case 6:
	//Get control gains values from PC
	procCmd6();
	break;
    case 7:
	//Get home positions from PC
	procCmd7();
	break;
    case 8:
	//Send current joint values to the PC
	procCmd8();
	break;
    case 9:
	//Emergency stop and reset
	procCmd9();
	break;
    case 10:
	//Reset encoder counts
	procCmd10();
	break;
    case 11:
	//Use as a misc function, now using it to test it as RS232
	procCmd11();
	break;
    case 12:
	//Calculate contact points on manipulator in terms of arclength
	procCmd12();
	break;
    case 13:
    //Get LQR gain matrix
    procCmd13();
    break;
    case 14:
    //Kill program
    procCmd14();
    break;
    case 15:
    //LQR Trajectory
    procCmd15();
    break;
    case 16:
	//Set trajectory control mode
	procCmd16();
	break;
    case -1:
	printf("Couldn't read a command\n");
	break;
    default:
	//printf("Bad command: %d\n", cmd);
	break;
    }

    //We are waiting for command again, should probably redo this with a semaphore
    waiting_for_cmd = 1;
}


//Get number of points for the trajectory from the PC and initialize arrays
void procCmd1() {
    sendString("ALLOCATE\n");
    num_pts = -1;
    //Make sure we aren't running our control
    running = 0;
    //Receive number of data points and initialize arrays
    readInt(&num_pts);
    //Initialize arrays
    if(num_pts < 1) {
	printf("Need at least 1 point for trajectories\n");
	return;
    }

    sendInt(&num_pts);
    printf("num_pts: %d\n",num_pts);
    traj1 = (double *)calloc(num_pts, sizeof(double));
    traj2 = (double *)calloc(num_pts, sizeof(double));
    traj3 = (double *)calloc(num_pts, sizeof(double));
    traj4 = (double *)calloc(num_pts, sizeof(double));
    position1 = (double *)calloc(num_pts, sizeof(double));
    position2 = (double *)calloc(num_pts, sizeof(double));
    position3 = (double *)calloc(num_pts, sizeof(double));
    controlVals1 = (double *)calloc(num_pts, sizeof(double));
    controlVals2 = (double *)calloc(num_pts, sizeof(double));
    controlVals3 = (double *)calloc(num_pts, sizeof(double));
    objectX = (double *)calloc(num_pts, sizeof(double));
    objectY = (double *)calloc(num_pts, sizeof(double));
    objectTh = (double *)calloc(num_pts, sizeof(double));
    cameraPosX = (double *)calloc(num_pts, sizeof(double));
    cameraPosY = (double *)calloc(num_pts, sizeof(double));
    cameraPosTh = (double *)calloc(num_pts, sizeof(double));
    desXAccel = (double *)calloc(num_pts, sizeof(double));
    desYAccel = (double *)calloc(num_pts, sizeof(double));
    desThAccel = (double *)calloc(num_pts, sizeof(double));
    desAccel1 = (double *)calloc(num_pts, sizeof(double));
    desAccel2 = (double *)calloc(num_pts, sizeof(double));
    desAccel3 = (double *)calloc(num_pts, sizeof(double));
    uffX = (double *)calloc(num_pts, sizeof(double));
    uffY = (double *)calloc(num_pts, sizeof(double));
    uffTh = (double *)calloc(num_pts, sizeof(double));
    KT11 = (double *)calloc(num_pts, sizeof(double));
    KT12 = (double *)calloc(num_pts, sizeof(double));
    KT13 = (double *)calloc(num_pts, sizeof(double));
    KT14 = (double *)calloc(num_pts, sizeof(double));
    KT15 = (double *)calloc(num_pts, sizeof(double));
    KT16 = (double *)calloc(num_pts, sizeof(double));
    KT17 = (double *)calloc(num_pts, sizeof(double));
    KT18 = (double *)calloc(num_pts, sizeof(double));
    KT21 = (double *)calloc(num_pts, sizeof(double));
    KT22 = (double *)calloc(num_pts, sizeof(double));
    KT23 = (double *)calloc(num_pts, sizeof(double));
    KT24 = (double *)calloc(num_pts, sizeof(double));
    KT25 = (double *)calloc(num_pts, sizeof(double));
    KT26 = (double *)calloc(num_pts, sizeof(double));
    KT27 = (double *)calloc(num_pts, sizeof(double));
    KT28 = (double *)calloc(num_pts, sizeof(double));
    KT31 = (double *)calloc(num_pts, sizeof(double));
    KT32 = (double *)calloc(num_pts, sizeof(double));
    KT33 = (double *)calloc(num_pts, sizeof(double));
    KT34 = (double *)calloc(num_pts, sizeof(double));
    KT35 = (double *)calloc(num_pts, sizeof(double));
    KT36 = (double *)calloc(num_pts, sizeof(double));
    KT37 = (double *)calloc(num_pts, sizeof(double));
    KT38 = (double *)calloc(num_pts, sizeof(double));
    loopTimes = (double *)calloc(num_pts, sizeof(double));
    //Check to make sure allocation went well
    //loopTimes is last thing allocated so if it wasn't allocated, we had a problem
    if(loopTimes == NULL)
    	sendString("MEMFAIL\n");
    else
    	sendString("MEMWIN\n");
}

//Receive joint trajectory from PC
void procCmd2() {
    int i;
    //Make sure we aren't running
    running = 0;
    //Let PC know we are ready to recieve trajectory
    sendString("SENDDATA\n");
    //Receive joint trajectory
    int doublesRec;
    doublesRec = getDoublePacket(traj1, num_pts);
    //printf("Expected %d doubles, got %d doubles\n",num_pts, doublesRec);

    sendString("DONETRAJ1\n");
    doublesRec = getDoublePacket(traj2, num_pts);
    //printf("Expected %d doubles, got %d doubles\n",num_pts, doublesRec);

    sendString("DONETRAJ2\n");
    doublesRec = getDoublePacket(traj3, num_pts);
    //printf("Expected %d doubles, got %d doubles\n",num_pts, doublesRec);

    //sendString("DONETRAJ3\n");
    sendString("DONETRAJ\n");
}

//Send back data from previous run and do a "soft reset"
//Might want to include delays in here
void procCmd3() {
    int i;
    //char outBuf[100];
    //Make sure we aren't running
    running = 0;
    ////Set control mode to 0, no control
    //control_mode = NO_CONTROL;
    //Reset global index
    globalIndex = -1;
    //Send data back
    sendString("START\n");
    sendDoublePacket(position1, num_pts);
    delay(1);
    sendDoublePacket(position2, num_pts);
    delay(1);
    sendDoublePacket(position3, num_pts);
    delay(1);
    sendDoublePacket(controlVals1, num_pts);
    delay(1);
    sendDoublePacket(controlVals2, num_pts);
    delay(1);
    sendDoublePacket(controlVals3, num_pts);
    delay(1);
    sendDoublePacket(cameraPosX, num_pts);
    delay(1);
    sendDoublePacket(cameraPosY, num_pts);
    delay(1);
    sendDoublePacket(cameraPosTh, num_pts);
    delay(1);
    sendDoublePacket(loopTimes, num_pts);
    delay(1);
   sendDoublePacket(objectX, num_pts);
   delay(1);
   sendDoublePacket(objectY, num_pts);
   delay(1);
   sendDoublePacket(objectTh, num_pts);
   delay(1);
   sendDoublePacket(desXAccel, num_pts);
   delay(1);
   sendDoublePacket(desYAccel, num_pts);
   delay(1);
   sendDoublePacket(desThAccel, num_pts);
   delay(1);
   sendDoublePacket(desAccel1, num_pts);
   delay(1);
   sendDoublePacket(desAccel2, num_pts);
   delay(1);
   sendDoublePacket(desAccel3, num_pts);
   delay(1);
   simpleReset();
}

//Go to home position
void procCmd4() {
    //Make sure we aren't running a trajectory
    running = 0;
    //Reset integral errors before going home
    errorInt1 = 0;
    errorInt2 = 0;
    errorInt3 = 0;
    //Control mode 1 is go home
//    control_mode = GO_HOME_JOINTS;
    globalIndex = -1;
     //Control mode 7 is dynamic grasp maintain pos
    //control_mode = DYNAMIC_GRASP_POS;
    //Go to manipulator home position
    control_mode = PID_MANIP_POS;
    //Tell PC we are starting to go home
    sendString("HOME\n");
    printf("Going home\n");
}

//Execute trajectory
void procCmd5() {
    //Tell PC we are going to execute trajectory
    sendString("EXECUTE\n");
    //Start trajectory control, running and should start indexing
    globalIndex = 0;
    running = 1;
    //Reset integral errors
    errorInt1 = 0;
    errorInt2 = 0;
    errorInt3 = 0;
    control_mode = trajectory_control_mode;
}

//Get control gains
void procCmd6() {
    //Make sure we aren't running
    running = 0;
    //Tell PC we are ready to get control gains
    sendString("CONTROL\n");
    //Get control gains from PC
    //goes proportional (1/2/3), derivative (1/2/3), then integral (1/2/3)
    double gains[15];
    getDoublePacket(gains,15);
    kp1 = gains[0]; kp2 = gains[1]; kp3 = gains[2];
    kd1 = gains[3]; kd2 = gains[4]; kd3 = gains[5];
    ki1 = gains[6]; ki2 = gains[7]; ki3 = gains[8];
    INNER_K1_14 = gains[9]; INNER_K2_14 = gains[10];
    INNER_K1_11 = gains[11]; INNER_K2_11 = gains[12];
    INNER_K1_8  = gains[13]; INNER_K2_8  = gains[14];
    /* readDouble(&kp1); readDouble(&kp2); readDouble(&kp3); */
    /* readDouble(&kd1); readDouble(&kd2); readDouble(&kd3); */
    /* readDouble(&ki1); readDouble(&ki2); readDouble(&ki3); */
    /* readDouble(&INNER_K1_14); readDouble(&INNER_K2_14); */
    /* readDouble(&INNER_K1_11); readDouble(&INNER_K2_11); */
    /* readDouble(&INNER_K1_8); readDouble(&INNER_K2_8); */
//    readDouble(&kp1curr); readDouble(&kp2curr); readDouble(&kp3curr);
//    readDouble(&kd1curr); readDouble(&kd2curr); readDouble(&kd3curr);
    //Tell PC we are done with control gains
    sendString("GAINSUPDATED\n");
}

//Get home positions from PC
void procCmd7() {
    //Make sure we aren't running
    running = 0;
    //read the home positions
    double homes[3];
    getDoublePacket(homes,3);
    home1 = homes[0];
    home2 = homes[1];
    home3 = homes[2];
    /* readDouble(&home1); */
    /* readDouble(&home2); */
    /* readDouble(&home3); */
    //Get object home positions
    /* readDouble(&objHomeX); */
    /* readDouble(&objHomeY); */
    /* readDouble(&objHomeTh); */
    //We got home positions
    sendString("HOMEUPDATED\n");
}

//Send live encoder positions
void procCmd8() {
    //	char buf[200];
    //	sprintf(buf, "%f\n", current_position_RH14(iobase, 0));
    //	sendString(buf);
    //	sprintf(buf, "%f\n", current_position_RH11(iobase, 0));
    //	sendString(buf);
    //	sprintf(buf, "%f\n", current_position_RH8(iobase, 0));
    //	sendString(buf);
    double temp[3];
    
    temp[0] = current_position_RH14(iobase, 0);
    temp[1] = current_position_RH11(iobase, 0);
    temp[2] = current_position_RH8(iobase, 0);
    sendDoublePacket(temp,3);
}

//Emergency stop and "reset"
void procCmd9() {
    //Tell PC we got the stop command
    sendString("STOP\n");
    running = 0;
    globalIndex = -1;
    control_mode = NO_CONTROL;
    set_control_RH8(iobase, 0);
    set_control_RH11(iobase, 0);
    set_control_RH14(iobase, 0);
    //Now reset
    simpleReset();
}

//Reset encoder positions to 0
void procCmd10() {
    //Reset encoders
    current_position_RH8(iobase, 1);
    current_position_RH11(iobase, 1);
    current_position_RH14(iobase, 1);
    sendString("ENCRESET\n");
}

//"Miscellaneous" function
void procCmd11() {
    /* if(visionCount >= NUM_SAMPLES) { */
    /*   //Tell PC we have timing data */
    /*   sendString("TIMING\n"); */
    /*   //Send back timing data */
    /*   int i; */
    /*   double temp; */
    /*   printf("sending lots of data\n"); */
    /*   for(i=0; i< NUM_SAMPLES; i++) { */
    /*     temp = (double)cameraTiming[i]; */
    /*     sendDouble(&temp); */
    /*   } */
    /*   printf("Done sending lots of data\n"); */
    /*   visionCount++; */
    /* } else { */
    /*   //Tell PC we don't have timing data yet */
    /*   sendString("NOTIMING\n"); */
    /*   //Now, send back current x/y coordinates */
    /*   double xTemp, yTemp; */
    /*   xTemp = (double)(xGlobal[0]); */
    /*   yTemp = (double)(yGlobal[0]); */
    /*   sendDouble(&xTemp); */
    /*   sendDouble(&yTemp); */
    /* } */
    //Send back objects in order
    /* sendString("OBJORDER\n"); */
    /* int i, tempInt; */
    /* double temp; */
    /* for(i = 0; i < desObj; i++) { */
    /* 	temp = (double)(xGlobal[i]); */
    /* 	sendDouble(&temp); */
    /* 	temp = (double)(yGlobal[i]); */
    /* 	sendDouble(&temp); */
    /* 	tempInt = (areaGlobal[i]); */
    /* 	sendInt(&tempInt); */
    /* } */
/*   sendString("JOINTANGLESCAM\n"); */
/*   double jointAngles[2]; */
/*   jointAngles[0] = calculateJointOneCamera();//atan2(yGlobal[0] - yGlobal[1], xGlobal[0] - xGlobal[1]); */
/*   jointAngles[1] = calculateJointTwoCamera();//atan2(yGlobal[1] - yGlobal[2], xGlobal[1] - xGlobal[2]) - jointAngles[0]; */
/*   sendDouble(&(jointAngles[0])); */
/*   sendDouble(&(jointAngles[1])); */
/*   int i; */
/*   double temp; */
/*   for(i = 0; i < 3; i++) { */
/* //      temp = (double)xGlobal[i]; */
/* //      sendDouble(&temp); */
/* //      temp = (double)yGlobal[i]; */
/* //      sendDouble(&temp); */
/* 	  sendDouble(&(xGlobal[i])); */
/* 	  sendDouble(&(yGlobal[i])); */
/*       sendInt(&(areaGlobal[i])); */
/*   } */


    //Test receiving a packet
    /* double buf[5]; */
    /* int doubles_rec = getDoublePacket(buf, 5); */
    /* printf("Received %d doubles, %f %f %f %f %f\n",doubles_rec, buf[0], buf[1], buf[2], buf[3], buf[4]); */
    /* sendDoublePacket(buf, doubles_rec); */
    double buf[3];
    buf[0] = xManip_global;
    buf[1] = yManip_global;
    buf[2] = thManip_global;
    sendDoublePacket(buf,3);
}

//Contact point on manipulator in terms of arc length
void procCmd12() {
//    arcLengthContactPoints();
    sendString("CONTACTPOINT\n");
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
        printf("xM: %f, yM: %f, thm: %f\n",xm,ym,thm);
        printf("xo: %f, yo: %f, tho: %f\n",xObjectGlobal, yObjectGlobal, thObjectGlobal);
        printf("s1: %f, s2: %f\n", contactPoint1, contactPoint2);
        printf("xRH14: %f, yRH14: %f, thRH14: %f\n", xGlobal[0], yGlobal[0], th1);
        printf("xRH11: %f, yRH11: %f, thRH11: %f\n", xGlobal[1], yGlobal[1], th2);
        printf("xRH8: %f, yRH8: %f, thRH8: %f\n", xGlobal[2], yGlobal[2], th3);
        printf("xMark1: %f, yMark1: %f\n",xGlobal[3], yGlobal[3]);
        printf("xMark2: %f, yMark2: %f\n",xGlobal[4], yGlobal[4]);
        printf("Area1: %d, Area2: %d, Area3: %d, Area4: %d, Area5: %d\n", areaGlobal[0], \
        		areaGlobal[1], areaGlobal[2], areaGlobal[3], areaGlobal[4]);
}

void procCmd13() {
    //Tell PC we are ready to get control gains
    sendString("LQRGAINS\n");
    //Get control gains from PC
    //goes proportional (1/2/3), derivative (1/2/3), then integral (1/2/3)
    int i;
    double gains[6];
    int bytesRec = getDoublePacket(K_lqr,24);
    bytesRec = getDoublePacket(gains,6);
    INNER_K1_14 = gains[0]; INNER_K2_14 = gains[1];
    INNER_K1_11 = gains[2]; INNER_K2_11 = gains[3];
    INNER_K1_8  = gains[4]; INNER_K2_8  = gains[5];
    sendString("LQRUPDATED\n");
}

void procCmd14() {
	//Tell PC
	sendString("KILL\n");
	end_program = 1;
}

//Receive LQR trajectory tracking gains
//Receive joint trajectory from PC
void procCmd15() {
    //Make sure we aren't running
    running = 0;
    //Let PC know we are ready to recieve trajectory
    sendString("SENDLQR\n");
    //Receive LQR feedback gains trajectory
    int doublesRec;
    //Get the four trajectories
    doublesRec = getDoublePacket(traj1, num_pts);
    sendString("DONETRAJ1\n");
    doublesRec = getDoublePacket(traj2, num_pts);
    sendString("DONETRAJ2\n");
    doublesRec = getDoublePacket(traj3, num_pts);
    sendString("DONETRAJ3\n");
    doublesRec = getDoublePacket(traj4, num_pts);
    sendString("DONETRAJ4\n");
    //Get feedforward terms
    doublesRec = getDoublePacket(uffX, num_pts);
    sendString("DONEFFX\n");
    doublesRec = getDoublePacket(uffY, num_pts);
    sendString("DONEFFY\n");
    doublesRec = getDoublePacket(uffTh, num_pts);
    sendString("DONEFFTH\n");
    doublesRec = getDoublePacket(KT11, num_pts);
    sendString("DONEK11\n");
    doublesRec = getDoublePacket(KT12, num_pts);
    sendString("DONEK12\n");
    doublesRec = getDoublePacket(KT13, num_pts);
    sendString("DONEK13\n");
    doublesRec = getDoublePacket(KT14, num_pts);
    sendString("DONEK14\n");
    doublesRec = getDoublePacket(KT15, num_pts);
    sendString("DONEK15\n");
    doublesRec = getDoublePacket(KT16, num_pts);
    sendString("DONEK16\n");
    doublesRec = getDoublePacket(KT17, num_pts);
    sendString("DONEK17\n");
    doublesRec = getDoublePacket(KT18, num_pts);
    sendString("DONEK18\n");
    doublesRec = getDoublePacket(KT21, num_pts);
    sendString("DONEK21\n");
    doublesRec = getDoublePacket(KT22, num_pts);
    sendString("DONEK22\n");
    doublesRec = getDoublePacket(KT23, num_pts);
    sendString("DONEK23\n");
    doublesRec = getDoublePacket(KT24, num_pts);
    sendString("DONEK24\n");
    doublesRec = getDoublePacket(KT25, num_pts);
    sendString("DONEK25\n");
    doublesRec = getDoublePacket(KT26, num_pts);
    sendString("DONEK26\n");
    doublesRec = getDoublePacket(KT27, num_pts);
    sendString("DONEK27\n");
    doublesRec = getDoublePacket(KT28, num_pts);
    sendString("DONEK28\n");
    doublesRec = getDoublePacket(KT31, num_pts);
    sendString("DONEK31\n");
    doublesRec = getDoublePacket(KT32, num_pts);
    sendString("DONEK32\n");
    doublesRec = getDoublePacket(KT33, num_pts);
    sendString("DONEK33\n");
    doublesRec = getDoublePacket(KT34, num_pts);
    sendString("DONEK34\n");
    doublesRec = getDoublePacket(KT35, num_pts);
    sendString("DONEK35\n");
    doublesRec = getDoublePacket(KT36, num_pts);
    sendString("DONEK36\n");
    doublesRec = getDoublePacket(KT37, num_pts);
    sendString("DONEK37\n");
    doublesRec = getDoublePacket(KT38, num_pts);
    sendString("DONEK38\n");

    INNER_K1_14 = INN_K1_14;
    INNER_K2_14 = INN_K2_14;
    INNER_K1_11 = INN_K1_11;
    INNER_K2_11 = INN_K2_11;
    INNER_K1_8 = INN_K1_8;
    INNER_K2_8 = INN_K2_8;
    printf("Got everything for LQR traj\n");
}

void procCmd16() {
    sendString("SETMODE\n");
    readInt(&trajectory_control_mode);
}

