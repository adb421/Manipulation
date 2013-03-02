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
  printf("Welcome to the 3 degree of freedom arm control!\n");

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
  pthread_setschedparam(tcpip_com_rec_thread_id, SCHED_RR, &tcpip_com_sched_params);
  pthread_setschedparam(tcpip_com_send_thread_id, SCHED_RR, &tcpip_com_sched_params);
  pthread_setschedparam(control_loop_thread_id, SCHED_RR, &control_loop_sched_params);
  pthread_setschedparam(vision_loop_thread_id, SCHED_RR, &vision_loop_sched_params);

  while(1) {
    //sched_yield(); This uses more kernel time, use sleep instead. Sleep for second at a time. Could change this.
    sleep(3);
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

  //Positions of the motor at beginning of control loop
  double tempPos1, tempPos2, tempPos3;
  //control values as desired currents
  double desCur1, desCur2, desCur3;

  //Used for timing. Initialize preLoop.
  ClockTime(CLOCK_REALTIME, NULL, &preLoop);

  while(1) {
    //Block thread until interrupt occurs
    //This interrupt happens every time the timer runs out
    InterruptWait(0,NULL);
 //   printf("%d\n", control_mode);

    //Now we do control loop!
    //Read encoders every cycle
    tempPos1 = current_position_RH14(iobase, 0);
    tempPos2 = current_position_RH11(iobase, 0);
    tempPos3 = current_position_RH8(iobase, 0);

    if((globalIndex < num_pts && globalIndex >= 0) && running) {
      //We are executing a trajectory, record pos
      //Done before calculating controls because controls uses position arrays
      position1[globalIndex] = tempPos1;
      position2[globalIndex] = tempPos2;
      position3[globalIndex] = tempPos3;
      if(newCameraData) {
	  cameraPosX[globalIndex] = xGlobal[2]; // Last object
	  cameraPosY[globalIndex] = yGlobal[2]; // Last object
	  cameraPos1[globalIndex] = calculateJointOneCamera();
	  cameraPos2[globalIndex] = calculateJointTwoCamera();
	  objectX[globalIndex] = xObjectGlobal;
	  objectY[globalIndex] = yObjectGlobal;
	  objectTh[globalIndex] = thObjectGlobal;
	  newCameraData = 0;
      } else {
	  cameraPosX[globalIndex] = 1000;
	  cameraPosY[globalIndex] = 1000;
	  cameraPos1[globalIndex] = 1000;
	  cameraPos2[globalIndex] = 1000;
	  //Can update this in the future to include a velocity term/estimate
	  objectX[globalIndex] = xObjectGlobal;
	  objectY[globalIndex] = yObjectGlobal;
	  objectTh[globalIndex] = thObjectGlobal;
      }
    }
    //calculate control, even if we aren't running a trajectory
    desCur1 = calculateControl1();
    desCur2 = calculateControl2();
    desCur3 = calculateControl3();

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
    if(globalIndex >=(num_pts -1) && control_mode != 7) {
      //No longer running
      running = 0;
      //Kill control
      control_mode = 0;
      //Reset globalIndex
      globalIndex = -1;
    }
    //Otherwise, increment and end control loop
    if((globalIndex < (num_pts -1)) && (globalIndex >= 0)) {
      globalIndex++;
    }

    //Check post loop times to see how long loop takes, if we are doing a traj. Otherwise don't care.
    if((globalIndex < num_pts && globalIndex >= 0) && running) {
      ClockTime(CLOCK_REALTIME, NULL, &postLoop);
      //How much time in nanoseconds has elapsed?
      nsecElapsed = postLoop - preLoop;
      //Is it the largest such elapsed time?
      loopTimes[globalIndex] = nsecElapsed;
      if(nsecElapsed > longestLoopTime)
	longestLoopTime = nsecElapsed;
    }
    //Record "preloop time"
    ClockTime(CLOCK_REALTIME, NULL, &preLoop);
  }
}
