#ifndef _LANDER_CONTROL_H
#define _LANDER_CONTROL_H
#include <stdio.h>
// Simulation parameters - YOU MUST NOT CHANGE ANY OF THESE
#define G_ACCEL 8.87
#define MT_ACCEL 35.0
#define RT_ACCEL 25.0
#define LT_ACCEL 25.0
#define MAX_ROT_RATE .075
#define SONAR_RANGE 9.0
#define NP1 .05
#define NP2 .05
#define T_STEP .005
#define S_SCALE 5.0
#define PI 3.14159265359
#define DISPLAY_LATENCY 10
#define HIST 180

// Global variables accessible to your flight computer
extern int MT_OK;
extern int RT_OK;
extern int LT_OK;
extern double PLAT_X;
extern double PLAT_Y;
extern double SONAR_DIST[36];

// Flight controls
void Main_Thruster(double power);
void Left_Thruster(double power);
void Right_Thruster(double power);
void Rotate(double angle);
double Velocity_X(void);
double Velocity_Y(void);
double Position_X(void);
double Position_Y(void);
double Angle(void);
double RangeDist(void);

// Function prototypes for code you need to look at

void Lander_Control(void);
void Safety_Override(void);
void stage1();
void stage2();
void set_angle_wrt_working_thruster();
void ascend_straight();
double Robust_Pos_X();
double Robust_Pos_Y();
void Thruster_and_Angle_Control();
void Apply_Thruster_Power();
double Robust_Angle();
double Robust_Vel_X();
double Robust_Vel_Y();
/*
void Lander_Control(void);
void Safety_Override(void);
void stage1();
void stage2();
void set_angle_wrt_working_thruster();
void ascend_straight();
void Check_Pos_X();
void Check_Pos_Y();
void Check_Vel_X();
void Check_Vel_Y();
void Check_Angle();
void Check_Sonar();
double Robust_Pos_X();
double Robust_Pos_Y();
double Robust_Vel_X();
double Robust_Vel_Y();
double Robust_Angle();
double Robust_Sonar();
*/
#endif
