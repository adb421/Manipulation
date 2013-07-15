/*
 * Adam Barber
 * NxR Lab
 * 11/29/2012
 * Last updated 1/14/2013
 * Main file for controlling the 3-degree of freedom arm on the qnx system
 * Talks to a matlab program for a better interface
 */
//Clean up my include files, may not be the best way to do it but I don't know
#include "includes.h"

//prototype for control loop thread
void * control_loop_thread(void *arg);

int main(int argc, char *argv[]) {
  //Just let us know we're running
  printf("Welcome to the 3 degree of freedom arm control! Maybe!\n");

  if(ThreadCtl(_NTO_TCTL_IO,0) == -1)
    printf("ThreadCtl fail\n");

  //pthread_attr_t attr;
  //pthread_attr_init(&attr);
  //pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

  //Change tick size to 20 us
  struct _clockperiod new;
  new.nsec = 20*1000; //20 us, in nanoseconds
  new.fract = 0;

  if(ClockPeriod(CLOCK_REALTIME, &new, NULL, 0) == -1)
    printf("Clock period change fail\n");

  //Do some initialization
  initialize_variables();
  iobase = initialize_sensoray();
  initialize_encoder_1(iobase);
  initialize_encoder_2(iobase);
  initialize_encoder_3(iobase);
  initialize_junus(iobase);

  //Start receive and send communication threads over tcp/ip with matlab
  pthread_t tcpip_com_rec_thread_id;
  pthread_create(&tcpip_com_rec_thread_id, NULL, tcpip_com_rec_thread, NULL);
  pthread_t tcpip_com_send_thread_id;
  pthread_create(&tcpip_com_send_thread_id, NULL, tcpip_com_send_thread, NULL);
  struct sched_param tcpip_com_sched_params, control_loop_sched_params, vision_loop_sched_params;
	
  pthread_t vision_loop_thread_id;
  pthread_create(&vision_loop_thread_id, NULL, vision_loop_thread, NULL);

  pthread_t control_loop_thread_id;
  pthread_create(&control_loop_thread_id, NULL, control_loop_thread, NULL);
	
  //Set thread priorities
  tcpip_com_sched_params.sched_priority = LOW_PRIORITY;
  vision_loop_sched_params.sched_priority = MEDIUM_PRIORITY;
  control_loop_sched_params.sched_priority = HIGH_PRIORITY;
  pthread_setschedparam(tcpip_com_rec_thread_id, SCHED_FIFO, &tcpip_com_sched_params);
  pthread_setschedparam(tcpip_com_send_thread_id, SCHED_FIFO, &tcpip_com_sched_params);
  pthread_setschedparam(control_loop_thread_id, SCHED_FIFO, &control_loop_sched_params);
  pthread_setschedparam(vision_loop_thread_id, SCHED_FIFO, &vision_loop_sched_params);

  while(!end_program) {
    //sched_yield(); This uses more kernel time, use sleep instead. Sleep for second at a time. Could change this.
    sleep(1);
  }
  return EXIT_SUCCESS;
}

void * control_loop_thread(void *arg) {

  //Set up our timer
  //Start with event, set it up to gen an interrupt
  struct sigevent controlLoopEvent;
  SIGEV_INTR_INIT(&controlLoopEvent);

  //Create a timer
  timer_t controlLoopTimer;
  timer_create(CLOCK_REALTIME, &controlLoopEvent, &controlLoopTimer);
  //Specify times for the timer for a pure periodic timer
  struct itimerspec controlLoopTimerSpec;
  controlLoopTimerSpec.it_value.tv_sec = 0;
  controlLoopTimerSpec.it_value.tv_nsec = LOOP_TIME_NSEC;
  controlLoopTimerSpec.it_interval.tv_sec = 0;
  controlLoopTimerSpec.it_interval.tv_nsec = LOOP_TIME_NSEC;
  //start the timer
  timer_settime(controlLoopTimer,0,&controlLoopTimerSpec, NULL);

  //timer structure pointer to check times
  _uint64 preLoop = 0;
  _uint64 postLoop = 0;
  //Number of nanoseconds passed
  _uint64 nsecElapsed = 0;
  _uint64 looptimesum = 0;

  //control values as desired currents
  double desCur1, desCur2, desCur3;
  int printTimer = 0;
  //Declare some variables for later
  double thRH14_prev, thRH11_prev, thRH8_prev, xManip_prev, yManip_prev, thManip_prev;
  double velRH14_prev, velRH11_prev, velRH8_prev, velXManip_prev, velYManip_prev, velThManip_prev;
  double xObject_prev, yObject_prev, thObject_prev;
  double velXObject_prev, velYObject_prev, velThObject_prev;
  double thObjCam, xObjCam, yObjCam;
  double diffX, diffY, normAngle, normDistObject;

  //Keep track of how many cycles its been since we got camera data
  int cameraFrameCount = 0;
  //Used for timing. Initialize preLoop.
  ClockTime(CLOCK_REALTIME, NULL, &preLoop);

  while(1) {
    //Block thread until interrupt occurs
    //This interrupt happens every time the timer runs out
    InterruptWait(0,NULL);
    //Flip that pin
    flip_pin(iobase);
 //   printf("%d\n", control_mode);
    ClockTime(CLOCK_REALTIME, NULL, &preLoop);
    //Now we do control loop!
    //Read encoders every cycle
    thRH14_prev = thRH14global;
    thRH11_prev = thRH11global;
    thRH8_prev = thRH8global;
    thRH14global = current_position_RH14(iobase, 0);
    thRH11global = current_position_RH11(iobase, 0);
    thRH8global = current_position_RH8(iobase, 0);
    //Update velocities
    velRH14_prev = velRH14global;
    velRH11_prev = velRH11global;
    velRH8_prev = velRH8global;
    //Apply RC lowpass filter
    velRH14global = (thRH14global - thRH14_prev)/DT*ALPHA_FILTER + (1.0-ALPHA_FILTER)*velRH14_prev;
    velRH11global = (thRH11global - thRH11_prev)/DT*ALPHA_FILTER + (1.0-ALPHA_FILTER)*velRH11_prev;
    velRH8global = (thRH8global - thRH8_prev)/DT*ALPHA_FILTER + (1.0-ALPHA_FILTER)*velRH8_prev;
    //Calculate manipulator positions
    xManip_prev = xManip_global;
    yManip_prev = yManip_global;
    thManip_prev = thManip_global;
    //Use forward kinematics to calculate manipulator end point position
    xManip_global = -1.0*L1*cos(thRH14global) - L2*cos(thRH14global+thRH11global);
    yManip_global = -1.0*L1*sin(thRH14global) - L2*sin(thRH14global+thRH11global);
    thManip_global = thRH14global + thRH11global + thRH8global;
    
    //Update velocities
    velXManip_prev = velXManip_global;
    velYManip_prev = velYManip_global;
    velThManip_prev = velThManip_global;
    velXManip_global = (xManip_global - xManip_prev)/DT*ALPHA_FILTER + (1.0 - ALPHA_FILTER)*velXManip_global;
    velYManip_global = (yManip_global - yManip_prev)/DT*ALPHA_FILTER + (1.0 - ALPHA_FILTER)*velYManip_global;
    velThManip_global = (thManip_global - thManip_prev)/DT*ALPHA_FILTER + (1.0 - ALPHA_FILTER)*velThManip_global;

    if(control_mode != NO_CONTROL) {
    	//Check limits
    	if(limitsExceeded())
    		control_mode = NO_CONTROL;
    }
    cameraFrameCount++;
    if(newCameraData) {
	//Calculate object positions
	xManipCam_global = xGlobal[2];
	yManipCam_global = yGlobal[2];
	if(fabs(xManipCam_global) > 10 || fabs(yManipCam_global) > 10)
		printf("Saw large val problem\n");
	xGlobal[3] = xGlobal[3] - xManipCam_global + xManip_global;
	xGlobal[4] = xGlobal[4] - xManipCam_global + xManip_global;
	yGlobal[3] = yGlobal[3] - yManipCam_global + yManip_global;
	yGlobal[4] = yGlobal[4] - yManipCam_global + yManip_global;
	xObjectGlobal = 0.5*(xGlobal[3] + xGlobal[4]);
	yObjectGlobal = 0.5*(yGlobal[3] + yGlobal[4]);
	thObjectGlobal = atan2(yGlobal[4] - yGlobal[3], xGlobal[4] - xGlobal[3]);
	//Is this right?
	if(thObjectGlobal - thObject_prev >= 3 && cameraFrameCount < 12)
	    thObjectGlobal -= 2.0*M_PI;
	else if(thObject_prev - thObjectGlobal >= 3 && cameraFrameCount < 12)
	    thObjectGlobal += 2.0*M_PI;

	normDistObject = sqrt(pow(xObjectGlobal - xManip_global,2) + pow(yObjectGlobal - yManip_global,2));
	if(normDistObject > 3.0*lc && control_mode == ONE_POINT_ROLL_BALANCE) {
		control_mode = 0;
		printf("Control killed, contact lost\n");
	}

	/* arcLengthContactPoints(); */
	velXObject_prev = velXObjectGlobal;
	velYObject_prev = velYObjectGlobal;
	velThObject_prev = velThObjectGlobal;
	//Figure out how long its been since we got camera data, should always be 4, but maybe not!
	velXObjectGlobal = (xObjectGlobal - xObject_prev)/(cameraFrameCount*DT)*ALPHA_FILTER_CAM + \
	    (1.0-ALPHA_FILTER_CAM)*velXObject_prev;
	velYObjectGlobal = (yObjectGlobal - yObject_prev)/(cameraFrameCount*DT)*ALPHA_FILTER_CAM + \
	    (1.0-ALPHA_FILTER_CAM)*velYObject_prev;
	velThObjectGlobal = (thObjectGlobal - thObject_prev)/(cameraFrameCount*DT)*ALPHA_FILTER_CAM + \
	    (1.0-ALPHA_FILTER_CAM)*velThObject_prev;
	xObject_prev = xObjectGlobal;
	yObject_prev = yObjectGlobal;
	thObject_prev = thObjectGlobal;
	newCameraData = 0;
	cameraFrameCount = 0;
    } else {
	//estimate new pos based on velocity
	xObjectGlobal += velXObjectGlobal*DT;
	yObjectGlobal += velYObjectGlobal*DT;
	thObjectGlobal += velThObjectGlobal*DT;
    }

    if((globalIndex < num_pts && globalIndex >= 0) && running) {
      //We are executing a trajectory, record pos
      //Done before calculating controls because controls uses position arrays
      position1[globalIndex] = thRH14global;
      position2[globalIndex] = thRH11global;
      position3[globalIndex] = thRH8global;
      objectX[globalIndex] = xObjectGlobal;
      objectY[globalIndex] = yObjectGlobal;
      objectTh[globalIndex] = thObjectGlobal;
//      cameraPosX[globalIndex] = xObjCam;
//      cameraPosY[globalIndex] = yObjCam;
//      cameraPosTh[globalIndex] = thObjCam;
      cameraPosX[globalIndex] = xManipCam_global;
      cameraPosY[globalIndex] = yManipCam_global;
      cameraPosTh[globalIndex] = thObjCam;
    }
    //calculate control, even if we aren't running a trajectory
    /* desCur1 = calculateControl1(); */
    /* desCur2 = calculateControl2(); */
    /* desCur3 = calculateControl3(); */
    calculateControl(&desCur1, &desCur2, &desCur3);
    
    //Set control values
    set_control_RH14(iobase, desCur1);
    set_control_RH11(iobase, desCur2);
    set_control_RH8(iobase, desCur3);

    if((globalIndex < num_pts && globalIndex >= 0) && running) {
      //Executing a trajectory, record control values
      controlVals1[globalIndex] = desCur1;
      controlVals2[globalIndex] = desCur2;
      controlVals3[globalIndex] = desCur3;
    }

    //Check if we finished traj
    if(globalIndex >=(num_pts -1) && num_pts > 1) {
      //No longer running
      running = 0;
      //Kill control
      control_mode = NO_CONTROL;
      errorInt1 = 0;
      errorInt2 = 0;
      errorInt3 = 0;
//      control_mode = PID_MANIP_POS;
      //Reset globalIndex
      globalIndex = -1;
    }
    //Otherwise, increment and end control loop
    if((globalIndex < (num_pts -1)) && (globalIndex >= 0)) {
      globalIndex++;
    }
//     printTimer++;
//     if(printTimer >= 2000 ) {
// //    	printf("Curr1: %f, curr2: %f, curr3: %f\n", desCur1, desCur2, desCur3); */
////     	printf("th1: %f, th2: %f, th3: %f\n",thRH14global, thRH11global,thRH8global);
// //    	printf("xm: %f, ym: %f, thm: :%f\n", xManip_global, yManip_global, thManip_global); */
// //    	printf("th1d: %f, th2d: %f, th3d: %f\n\n", velRH14global, velRH11global, velRH8global); */
//     	printf("xObj: %f, yObj: %f, thObj: %f\n", xObjectGlobal, yObjectGlobal, thObjectGlobal);
//     	printf("velXObj: %f, velYObj: %f, velthObj: %f\n", velXObjectGlobal, velYObjectGlobal, velThObjectGlobal);
//    	printf("xManip_enc: %f, yManip_enc: %f, thManip_enc: %f\n", xManip_global, yManip_global, thManip_global);
//    	printf("xManip_cam: %f, yManip_cam: %f\n", xGlobal[2], yGlobal[2]);
//    	printf("velXMan: %f, velYMan: %f, velThMan: %f\n", velXManip_global, velYManip_global, velThManip_global);
//     	printf("curr1: %f, curr2: %f, curr3: %f\n",desCur1, desCur2, desCur3);
//     	printf("Avg loop time: %f\n\n",looptimesum/1000000.0/2000.0);
//     	looptimesum = 0;
//     	printTimer = 0;
//     }



    //Check post loop times to see how long loop takes, if we are doing a traj. Otherwise don't care.
     ClockTime(CLOCK_REALTIME, NULL, &postLoop);
    if((globalIndex < num_pts && globalIndex >= 0) && running) {
      //How much time in nanoseconds has elapsed?
      nsecElapsed = postLoop - preLoop;
      //Is it the largest such elapsed time
      loopTimes[globalIndex] = nsecElapsed;
      if(nsecElapsed > longestLoopTime)
	longestLoopTime = nsecElapsed;
    }
//    looptimesum += postLoop - preLoop;
    //Record "preloop time"
  }
}
