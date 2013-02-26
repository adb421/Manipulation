/*
 * process_commands.c
 *
 *  Created on: Oct 30, 2012
 *      Author: Adam
 */

#include "process_commands.h"

void process_cmd() {
    //Changing to use serial
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
    sendString("GETTRAJ\n");
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
    char buf[100];
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
    loopTimes = (_uint64 *)calloc(num_pts, sizeof(_uint64));
    cameraPosX = (double *)calloc(num_pts, sizeof(double));
    cameraPosY = (double *)calloc(num_pts, sizeof(double));
    cameraPos1 = (double *)calloc(num_pts, sizeof(double));
    cameraPos2 = (double *)calloc(num_pts, sizeof(double));
    //Check to make sure allocation went well
    if(controlVals3 != NULL) {
	sendString("MEMWIN\n");
    } else {
	sendString("MEMFAIL\n");
    }
}

//Receive joint trajectory from PC
void procCmd2() {
    int i;
    //Make sure we aren't running
    running = 0;
    //Let PC know we are ready to recieve trajectory
    sendString("SENDDATA\n");
    //Receive joint trajectory
    for(i = 0; i < num_pts; i++) {
	readDouble(&(traj1[i]));
    }
    sendString("DONETRAJ1\n");
    for(i = 0; i < num_pts; i++) {
	readDouble(&(traj2[i]));
    }
    sendString("DONETRAJ2\n");
    for(i = 0; i < num_pts; i++) {
	readDouble(&(traj3[i]));
    }
    sendString("DONETRAJ3\n");
}

//Send back data from previous run and do a "soft reset"
//Might want to include delays in here
void procCmd3() {
    int i;
    char outBuf[100];
    //Make sure we aren't running
    running = 0;
    //Set control mode to 0, no control
    control_mode = 0;
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
    sendString("POSEND\n");
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
    double tempD;
    for(i = 0; i < num_pts; i++) {
	//		sprintf(outBuf, "%ull\n", loopTimes + i);
	//		sendString(outBuf);
	tempD = ((double)(loopTimes[i]))/1000000.0;
	sendDouble(&tempD);
    }
    sendString("END\n");
    //Send back camera x pos, y pos, joint angles 1 and 2
    for(i = 0; i < num_pts; i++) {
	sendDouble(cameraPosX + i);
    }
    for(i = 0; i < num_pts; i++) {
	sendDouble(cameraPosY + i);
    }
    for(i = 0; i < num_pts; i++) {
	sendDouble(cameraPos1 + i);
    }
    for(i = 0; i < num_pts; i++) {
	sendDouble(cameraPos2 + i);
    }
    simpleReset();
}

//Go to home position
void procCmd4() {
    //Make sure we aren't running a trajectory
    running = 0;
    //Reset integral errorsbefore going home
    errorInt1 = 0;
    errorInt2 = 0;
    errorInt3 = 0;
    //Control mode 1 is go home
    control_mode = 1;
    globalIndex = -1;
    //Tell PC we are starting to go home
    sendString("HOME\n");
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
    //control_mode = 3;
    //Trajectories given are desired currents
    //control_mode = 2;
    //Pure PID control
    //control_mode = 4;
    //Feedforward + PID control
    control_mode = 5;
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
    control_mode = 0;
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
  for(i = 0; i < desObj; i++) {
      temp = (double)xGlobal[i];
      sendDouble(&temp);
      temp = (double)yGlobal[i];
      sendDouble(&temp);
      sendInt(&(areaGlobal[i]));
  }
}

