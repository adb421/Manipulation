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
    case -1:
	printf("Couldn't read a command\n");
	break;
    default:
	printf("Bad command: %d\n", cmd);
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
//    char buf[100];
    //	sprintf(buf, "%d\n",num_pts);
    //	sendString(buf);
    sendInt(&num_pts);
    printf("num_pts: %d\n",num_pts);
    traj1 = (double *)calloc(num_pts, sizeof(double));
    traj2 = (double *)calloc(num_pts, sizeof(double));
    traj3 = (double *)calloc(num_pts, sizeof(double));
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
    loopTimes = (_uint64 *)calloc(num_pts, sizeof(_uint64));
    //Check to make sure allocation went well
    if(traj1 == NULL || traj2 == NULL || traj3 == NULL || \
    		position1 == NULL || position2 == NULL || position3 == NULL ||\
    		controlVals1 == NULL || controlVals2 == NULL || controlVals3 == NULL ||\
    		objectX == NULL || objectY == NULL || objectTh == NULL ||\
    		cameraPosX == NULL || cameraPosY == NULL || cameraPosTh == NULL || \
    		loopTimes == NULL)
    	sendString("MEMFAIL\n");
    else
    	sendString("MEMWIN\n");
}

//Receive joint trajectory from PC
void procCmd2() {
    int i;
    //Make sure we aren't running
    running = 0;
    double maxVal = -10.0;
    double minVal = 0.0;
    //Let PC know we are ready to recieve trajectory
    sendString("SENDDATA\n");
    //Receive joint trajectory
    for(i = 0; i < num_pts; i++) {
    	readDouble(&(traj1[i]));
    	if(traj1[i] > maxVal)
    		maxVal = traj1[i];
    	if(traj1[i] < minVal)
			minVal = traj1[i];
    }
    printf("Max traj1: %f\nMin traj1: %f\nFirst traj1: %f\n",maxVal,minVal,traj1[0]);
    maxVal = -10.0;
    minVal = 0.0;
    sendString("DONETRAJ1\n");
    for(i = 0; i < num_pts; i++) {
	readDouble(&(traj2[i]));
	if(traj2[i] > maxVal)
		maxVal = traj2[i];
	if(traj2[i] < minVal)
		minVal = traj2[i];
    }
    printf("Max traj2: %f\nMin traj2: %f\nFirst traj2: %f\n",maxVal,minVal,traj2[0]);
    maxVal = -10.0;
    minVal = 0.0;
    sendString("DONETRAJ2\n");
    for(i = 0; i < num_pts; i++) {
	readDouble(&(traj3[i]));
	if(traj3[i] > maxVal)
		maxVal = traj3[i];
	if(traj3[i] < minVal)
		minVal = traj3[i];
    }
    printf("Max traj3: %f\nMin traj3: %f\n",maxVal,minVal,traj3[0]);
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
    //Send data back (sendData();
    sendString("START\n");
     for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%f\n", position1[i]);
 	//		sendString(outBuf);
 	sendDouble(position1 + i);
     }
     for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%f\n", position2[i]);
 	//		sendString(outBuf);
 	sendDouble(position2 + i);
     }
     for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%f\n", position3[i]);
 	//		sendString(outBuf);
 	sendDouble(position3 + i);
     }
 //    sendString("POSEND\n");
     for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%f\n", controlVals1[i]);
 	//		sendString(outBuf);
 	sendDouble(controlVals1 + i);
     }
     for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%f\n", controlVals2[i]);
 	//		sendString(outBuf);
 	sendDouble(controlVals2 + i);
     }
     for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%f\n", controlVals3[i]);
 	//		sendString(outBuf);
 	sendDouble(controlVals3 + i);
     }
     for(i = 0; i < num_pts; i++) {
     	sendDouble(cameraPosX + i);
     }
     for(i = 0; i < num_pts; i++) {
     	sendDouble(cameraPosY + i);
     }
     for(i = 0; i < num_pts; i++) {
 	sendDouble(cameraPosTh + i);
     }
    double tempD;
    for(i = 0; i < num_pts; i++) {
 	//		sprintf(outBuf, "%ull\n", loopTimes + i);
 	//		sendString(outBuf);
 	tempD = ((double)(loopTimes[i]))/1000000.0;
 	sendDouble(&tempD);
    }
   for(i = 0; i < num_pts; i++) {
       sendDouble(objectX + i);
   }
   for(i = 0; i < num_pts; i++) {
       sendDouble(objectY + i);
   }
   for(i = 0; i < num_pts; i++) {
       sendDouble(objectTh + i);
   }
   for(i = 0; i < num_pts; i++) {
       sendDouble(desXAccel + i);
   }
   for(i = 0; i < num_pts; i++) {
       sendDouble(desYAccel + i);
   }
   for(i = 0; i < num_pts; i++) {
       sendDouble(desThAccel + i);
   }
   for(i = 0; i < num_pts; i++) {
	   sendDouble(desAccel1 + i);
   }
   for(i = 0; i < num_pts; i++) {
	   sendDouble(desAccel2 + i);
   }
   for(i = 0; i < num_pts; i++) {
	   sendDouble(desAccel3 + i);
   }
    sendString("END\n");
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
    //These are different control modes
    //Feedforward control only
    //control_mode = FEEDFORWARD_JOINTS;
    //Trajectories given are desired currents
    //control_mode = TRAJ_IS_CURRENT;
    //Pure PID control
    //control_mode = PID_TRAJ_JOINTS;
    //Feedforward + PID control
   //control_mode = FF_PID_TRAJ_JOINTS;
    //Dynamic Grasp
 //   control_mode = DYNAMIC_GRASP_TRAJ;
    //Follow a manipulator trajectory
    //Go back to this!!!!!!!!
    control_mode = FF_PID_TRAJ_MANIP;
    //BALANCE THAT ISH
//    control_mode = ONE_POINT_ROLL_BALANCE;
//    home1 = traj1[num_pts - 1];
//    home2 = traj2[num_pts - 1];
//    home3 = traj3[num_pts - 1];
    //Right now, want to get estimate of gravity!
    //control_mode = NO_CONTROL;
}

//Get control gains
void procCmd6() {
    //Make sure we aren't running
    running = 0;
    //Tell PC we are ready to get control gains
    sendString("CONTROL\n");
    //Get control gains from PC
    //goes proportional (1/2/3), derivative (1/2/3), then integral (1/2/3)
    readDouble(&kp1); readDouble(&kp2); readDouble(&kp3);
    readDouble(&kd1); readDouble(&kd2); readDouble(&kd3);
    readDouble(&ki1); readDouble(&ki2); readDouble(&ki3);
    readDouble(&INNER_K1_14); readDouble(&INNER_K2_14);
    readDouble(&INNER_K1_11); readDouble(&INNER_K2_11);
    readDouble(&INNER_K1_8); readDouble(&INNER_K2_8);
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
    readDouble(&home1);
    readDouble(&home2);
    readDouble(&home3);
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
    double temp;
    temp = current_position_RH14(iobase, 0);
    sendDouble(&temp);
    temp = current_position_RH11(iobase, 0);
    sendDouble(&temp);
    temp = current_position_RH8(iobase, 0);
    sendDouble(&temp);
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
  sendString("JOINTANGLESCAM\n");
  double jointAngles[2];
  jointAngles[0] = calculateJointOneCamera();//atan2(yGlobal[0] - yGlobal[1], xGlobal[0] - xGlobal[1]);
  jointAngles[1] = calculateJointTwoCamera();//atan2(yGlobal[1] - yGlobal[2], xGlobal[1] - xGlobal[2]) - jointAngles[0];
  sendDouble(&(jointAngles[0]));
  sendDouble(&(jointAngles[1]));
  int i;
  double temp;
  for(i = 0; i < 3; i++) {
//      temp = (double)xGlobal[i];
//      sendDouble(&temp);
//      temp = (double)yGlobal[i];
//      sendDouble(&temp);
	  sendDouble(&(xGlobal[i]));
	  sendDouble(&(yGlobal[i]));
      sendInt(&(areaGlobal[i]));
  }
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
    for(i = 0; i < 24; i++) {
    	readDouble(&(K_lqr[i]));
    }
    sendString("LQRUPDATED\n");
}

void procCmd14() {
	//Tell PC
	sendString("KILL\n");
	end_program = 1;
}
