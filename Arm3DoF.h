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
#define PID_MANIP_POS      9  //Go to a specified manipulator pos aka home position
#define ONE_POINT_ROLL_BALANCE 10 // Balance one point rolling, stabilize a point
#define NEW_ONE_POINT_ROLL_BALANCE 11 //Balance one point rolling, LQR is on manipulator config instead of object

#define LOW_PRIORITY	30
#define MEDIUM_PRIORITY 35
#define HIGH_PRIORITY	40

//Horizontal is 0
#define TABLE_ANGLE		 (0.4) //Table angle is approx 22 degrees right now

#define MAX_CURRENT_RH14 (5.4) //Amps
#define MAX_CURRENT_RH11 (2.1) //Amps
#define MAX_CURRENT_RH8  (1.6)//Amps

#define LOOP_TIME_NSEC (1000000) //1ms
#define DT			   (0.001) //time in seconds

//Object parameters
//#define SQUARE_OBJECT
#define RECT_OBJECT
#ifdef SQUARE_OBJECT
#define lo (0.05) //10cm, divide by 2, in m
#define wo (0.05)
#define mo (0.06) //60 g
#define Io ((mo*(4.0*lo*lo + 4.0*wo*wo))/12.0) // m(h^2 + w^2)/12
#define OBJECT_ANGLE (M_PI/4.0) //atan(wo/lo)
#endif
#ifdef RECT_OBJECT
#define wo (.026) //5.2cm, divide by 2, 2.6
#define lo (.0425)  //8.5cm, divide by 2, 4.25
#define mo (0.088) // 51 g
#define Io ((mo*(4.0*lo*lo + 4.0*wo*wo))/12.0) // m(h^2 + w^2)/12
#define lc (0.04956)
#define OBJECT_ANGLE (0.5490) //atan(wo/lo)
#endif

//LQR Point Roll Balance points
#define X_MANIP_ROLL_GOAL (-0.2)
#define Y_MANIP_ROLL_GOAL (-0.1) //params.wm & params.lc ignore them for now
#define X_OBJ_ROLL_GOAL (-0.2)
#define Y_OBJ_ROLL_GOAL (-0.1)
#define TH_OBJ_ROLL_GOAL (M_PI/2.0 - OBJECT_ANGLE)

//LQR gains, do it this way for now IM LAZY
//#define K11 (0.0083)
//#define K12 (0.0221)
//#define K13 (-9.997)
//#define K14 (-16.5494)
//#define K15 (0.0)
//#define K16 (0.0)
//#define K17 (47.5807)
//#define K18 (10.5380)
//
//#define K21 (0.0)
//#define K22 (0.0)
//#define K23 (0.0)
//#define K24 (0.0)
//#define K25 (10.0)
//#define K26 (10.9545)
//#define K27 (0.0)
//#define K28 (0.0)
//
//#define K31 (0.3161)
//#define K32 (0.8557)
//#define K33 (0.2613)
//#define K34 (0.4415)
//#define K35 (0.0)
//#define K36 (0.0)
//#define K37 (1.2101)
//#define K38 (-0.2687)
//Try this:
#define K11 (0.191)
#define K12 (0.0621)
#define K13 (-0.9998)
#define K14 (-2.0466)
#define K15 (0.0)
#define K16 (0.0)
#define K17 (9.9146)
#define K18 (1.4487)
#define K21 (0.0)
#define K22 (0.0)
#define K23 (0.0)
#define K24 (0.0)
#define K25 (1.0)
#define K26 (1.7321)
#define K27 (0.0)
#define K28 (0.0)
#define K31 (0.9998)
#define K32 (1.7312)
#define K33 (0.0191)
#define K34 (0.0624)
#define K35 (0.0)
#define K36 (0.0)
#define K37 (-0.2239)
#define K38 (0.0336)

//From motor data sheet
#define KM1		(2.92)
#define KM2		(4.91)
#define KM3		(2.10)
#define J1		(0.0216)
#define J2		(0.043)
#define J3		(0.0037)
//Static friction model seems to be causing problems
#define MUS1	(1.2556/2.0)
#define MUS2	(1.5221/2.0)
#define MUS3	(0.504/10.0)
//#define MUS1 0.0
//#define MUS2 0.0
//#define MUS3 0.0
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
#define lm (0.09) // 18cm, divide by 2, in m
#define wm (0.0255) // 5.2cm, divide by 2, in m
#define m3 (0.1146 + M_MOUNT) // 114.6g
#define I3 (((m3-M_MOUNT)*(4.0*lm*lm + 4.0*wm*wm))/12.0) + I_MOUNT // m(h^2 + w^2)/ 12
#endif

//Home PI controls
#define KPH1 (4.0)
#define KIH1 (5.0)
#define KPH2 (3.0)
#define KIH2 (10.0)
#define KPH3 (1.0)
#define KIH3 (0.1)

#define IRQ4	4

//Aggressive filter time, 5Hz cutoff
#define ALPHA_FILTER (0.03)
#define ALPHA_FILTER_CAM (0.112)

//#define ALPHA_FILTER (0.15)
//#define ALPHA_FILTER_CAM (0.25)
//#define ALPHA_FILTER_CAM 1.0
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
//double feedForward1();
//double feedForward2();
//double feedForward3();
void set_control_RH8(uintptr_t iobase, double desCurrent);
void set_control_RH11(uintptr_t iobase, double desCurrent);
void set_control_RH14(uintptr_t iobase, double desCurrent);
void initialize_variables();
void initialize_junus(uintptr_t iobase);
void simpleReset();
void dynamicGraspControl(double xodd, double yodd, double thodd, double *xmdd, double *ymdd, double *thmdd);
void robotTorques(double *torqueDes1, double *torqueDes2, double *torqueDes3, \
		  double th1ddot, double th2ddot, double th3ddot, \
		  double th1dot, double th2dot, double th3dot, \
		  double th1, double th2, double th3);
void calculateJointAccelFromManipAccel(double xmdd, double ymdd, double thmdd, \
				       double *th1dd, double *th2dd, double *th3dd);
int limitsExceeded();
void flip_pin(uintptr_t iobase);


uintptr_t iobase;
uint16_t  DIO_word;

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
double K_lqr[24];

double errorInt1, errorInt2, errorInt3;

double *position1, *position2, *position3;
double *traj1, *traj2, *traj3;
double *controlVals1, *controlVals2, *controlVals3;
double *desXAccel, *desYAccel, *desThAccel;
double *objectX, *objectY, *objectTh;
_uint64 *loopTimes;

double *cameraPosX, *cameraPosY, *cameraPos1, *cameraPos2, *cameraPosTh;
double *desAccel1, *desAccel2, *desAccel3;

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

//These will eventually be #defs
double INNER_K1_14, INNER_K2_14;
double INNER_K1_11, INNER_K2_11;
double INNER_K1_8, INNER_K2_8;

short end_program;
#endif /* ARM3DOF_H_ */
