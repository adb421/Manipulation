/*
 * Arm3DoF.h
 *
 *  Created on: Oct 16, 2012
 *      Author: Adam
 */

#ifndef ARM3DOF_H_
#define ARM3DOF_H_

#include "includes.h"

//Control Modes
#define NO_CONTROL         0  //apply no torques
#define GO_HOME_JOINTS     1  //go to a specified joint home position
#define TRAJ_IS_CURRENT    2  //trajectories given are desired motor currents
#define FEEDFORWARD_JOINTS 3  //Feed forward only trajectory control, given as joint angle trajs
#define PID_TRAJ_JOINTS    4  //PID trajectory control, given as joint angle trajs
#define FF_PID_TRAJ_JOINTS 5  //Same as above, but with feed forward added
#define DYNAMIC_GRASP_TRAJ 6  //Follow a trajectory assuming dynamic grasp, trajs are object pos
#define DYNAMIC_GRASP_POS  7  //Go to a specified object pos, assume dyn grasp
#define FF_PID_TRAJ_MANIP  8  //Follow a trajectory, trajs are manipulator pos (x,y,th)
#define PID_MANIP_POS      9  //Go to a specified manipulator pos

#define LOW_PRIORITY	30
#define MEDIUM_PRIORITY 35
#define HIGH_PRIORITY	40

//Horizontal is 0
#define TABLE_ANGLE		 (M_PI/6) //Table angle is approx 30degrees right now

#define MAX_CURRENT_RH14 (5.4) //Amps
#define MAX_CURRENT_RH11 (2.1) //Amps
#define MAX_CURRENT_RH8  (1.6)//Amps

#define LOOP_TIME_NSEC (1000000) //1ms
#define DT			   (0.001) //time in seconds

//From motor data sheet
#define KM1		(2.92)
#define KM2		(4.91)
#define KM3		(2.10)
#define J1		(0.0216)
#define J2		(0.043)
#define J3		(0.0037)
#define MUS1	(1.2556)
#define MUS2	(1.522)
#define MUS3	(0.504)
#define MUD1	(0.035*30.0/M_PI)
#define MUD2	(0.017*30.0/M_PI)
#define MUD3	(0.0097*30.0/M_PI)

//Robot parameters
#define L1		(0.193675)
#define L2		(0.19685)
//#define L3		0.195
#define mm2		(0.489)
#define mm3		(0.35)
#define m1		(0.387)
#define m2		(0.193)
//#define m3		0.0405
#define g		(9.81*sin(TABLE_ANGLE))
#define I1		(m1*L1*L1/12.0)
#define I2		(m2*L2*L2/12.0)
//#define I3 		(m3*L3*L3/12.0)

//Manipulator parameters
//Always have the mounting bar
#define W_MOUNT (0.026) //26mm
#define L_MOUNT (0.205) //20.5cm
#define M_MOUNT (0.0276) //27.6g
#define I_MOUNT (M_MOUNT*(L_MOUNT*L_MOUNT + W_MOUNT*W_MOUNT)/12.0)
#define RECT_MANIP
#ifdef RECT_MANIP
#define lm (0.115) // 23cm, divide by 2, in m
#define wm (0.04) // 8cm, divide by 2, in m
#define m3 (0.125 + M_MOUNT) // 125g
#define I3 (((m3-M_MOUNT)*(4.0*lm*lm + 4.0*wm*wm))/12.0) + I_MOUNT // m(h^2 + w^2)/ 12
#endif

//Object parameters
#define SQUARE_OBJECT
#ifdef SQUARE_OBJECT
#define lo (0.05) //10cm, divide by 2, in m
#define wo (0.05)
#define mo (0.06) //60 g
#define Io ((mo*(4.0*lo*lo + 4.0*wo*wo))/12.0); // m(h^2 + w^2)/12
#endif

//Home PI controls
#define KPH1 (4.0)
#define KIH1 (5.0)
#define KPH2 (3.0)
#define KIH2 (10.0)
#define KPH3 (1.0)
#define KIH3 (0.1)

#define IRQ4	4

#define ALPHA_FILTER (0.225)
#define ALPHA_FILTER_CAM (0.35)
double current_position_RH8(uintptr_t iobase, int reset);
double current_position_RH11(uintptr_t iobase, int reset);
double current_position_RH14(uintptr_t iobase, int reset);
double calculateControl1();
double calculateControl2();
double calculateControl3();
double calculateTrajVel1();
double calculateTrajVel2();
double calculateTrajVel3();
double calculateTrajAccel1();
double calculateTrajAccel2();
double calculateTrajAccel3();
double feedForward1();
double feedForward2();
double feedForward3();
void set_control_RH8(uintptr_t iobase, double desCurrent);
void set_control_RH11(uintptr_t iobase, double desCurrent);
void set_control_RH14(uintptr_t iobase, double desCurrent);
void initialize_variables();
void initialize_junus(uintptr_t iobase);
void simpleReset();
double controlManipAccel1(double xmdd, double ymdd, double thmdd);
double controlManipAccel2(double xmdd, double ymdd, double thmdd);
double controlManipAccel3(double xmdd, double ymdd, double thmdd);
void dynamicGraspControl(double xodd, double yodd, double thodd, double *xmdd, double *ymdd, double *thmdd);
double calculateJointAccelFromManipAccel1(double xmdd, double ymdd, double thmdd);
double calculateJointAccelFromManipAccel2(double xmdd, double ymdd, double thmdd);
double calculateJointAccelFromManipAccel3(double xmdd, double ymdd, double thmdd);
void robotTorques(double *torqueDes1, double *torqueDes2, double *torqueDes3, \
		  double th1ddot, double th2ddot, double th3ddot, \
		  double th1dot, double th2dot, double th3dot, \
		  double th1, double th2, double th3);
void calculateJointAccelFromManipAccel(double xmdd, double ymdd, double thmdd, \
				       double *th1dd, double *th2dd, double *th3dd);
uintptr_t iobase;
uint16_t  DIO_word;

int currRH14;
int currRH11;
int currRH8;

int lastRH14;
int lastRH11;
int lastRH8;

long long total_countRH14;
long long total_countRH11;
long long total_countRH8;

int control_mode;
int globalIndex;
int num_pts;
int running;

double home1, home2, home3;
double objHomeX, objHomeY, objHomeTh;

_uint64 longestLoopTime;

double kp1, kp2, kp3;
double kd1, kd2, kd3;
double ki1, ki2, ki3;
//double kp1curr, kp2curr, kp3curr;
//double kd1curr, kd2curr, kd3curr;

double error1, error2, error3;
double errord1, errord2, errord3;
double errorInt1, errorInt2, errorInt3;

double *position1, *position2, *position3;
double *traj1, *traj2, *traj3;
double *controlVals1, *controlVals2, *controlVals3;
double *objectX, *objectY, *objectTh;
_uint64 *loopTimes;

double *cameraPosX, *cameraPosY, *cameraPos1, *cameraPos2;
//double *desAccel1, *desAccel2, *desAccel3;

double contactPoint1, contactPoint2;

double thRH14global, thRH11global, thRH8global;
double xManip_global, yManip_global, thManip_global;
double velRH14global;
double velRH11global;
double velRH8global;
double velThManip_global, velXManip_global, velYManip_global;
double xObjectGlobal, yObjectGlobal, thObjectGlobal;
double velXObjectGlobal, velYObjectGlobal, velThObjectGlobal;
double xManipCam_global, yManipCam_global;
#endif /* ARM3DOF_H_ */
