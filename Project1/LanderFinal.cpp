/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 5 m/s at touchdown
	  * Maximum landing angle should be less than 10 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
static double error_accumulator_s1 = 0;
static double error_accumulator_s3 = 0;
static double error_accumulator_ascend_straight = 0;
static bool if_passed_s1 = false;
static bool if_safe = false;
static double angle_wrt_thru;
static int mode;
static 	double angle_wrt_working_thru;



static double Prev_X_Pos; // The the position of X at the previous second
static double Prev_X_V ; // The velocity in X direction at the previous second;

static double Prev_Y_Pos; // The the position of Y at the previous second
static double Prev_Y_V ; // The velocity in Y direction at the previous second;


static bool is_malfunc_pos_x = false;
static bool is_malfunc_pos_y = false;
static double Pos_X;
static double Pos_Y;
static int t = 0;

//static double reference_s2 = PLAT_X; // Set the Platform_X() to be the reference of  the stage 2;
#include <math.h>

#include "Lander_Control.h"

void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/
	if( t == 0)
	{
		Pos_X = Position_X();
		Prev_X_Pos = Pos_X;
		Prev_X_V = Velocity_X();
		Pos_Y = Position_Y();
		Prev_Y_Pos = Pos_Y;
		Prev_Y_V = Velocity_Y();
		t++;
		is_malfunc_pos_x = true;
	}
	else if( t > 0)
	{
		if(!is_malfunc_pos_x) check_pos_x_sensor();
		if(!is_malfunc_pos_x) 
		{
			Pos_X = Position_X();
		}
		else
		{
		 	Robust_Pos_X();
		}
		
		if(!is_malfunc_pos_y) check_pos_y_sensor();
		if(!is_malfunc_pos_y) 
		{
			Pos_Y = Position_Y();
		}
		else
		{
		 	Robust_Pos_Y();
		}
	}
	//
	printf("estimated = %f, exact = %f Vel %f \n", Pos_X, Position_X(), Velocity_X() ); 
	printf("estimatedY = %f, exactY = %f VelY %f \n", Pos_Y, Position_Y(), Velocity_Y() ); 
	
	 // If the main thruster works, we change mode to be 0.
	 if(MT_OK)
	 {
	 	mode = 0;
	 }
	 // Otherwise, if the left thruster is working, we change the mode to be 1.
	 else if(LT_OK)
	 {
	 	mode = 1;
	 }
	 // Otherwise, the mode will be changed to be 2.
	 else if(RT_OK)
	 {
	 	mode = 2;
	 }
	set_angle_wrt_working_thruster();
	if(!if_passed_s1)
	{
		stage1();
	}
	else
	{
		stage2();
	}
	Prev_X_Pos = Pos_X;
	Prev_X_V = Velocity_X();
	Prev_Y_Pos = Pos_Y;
	Prev_Y_V = Velocity_Y();

}



void stage1()
{	


	 double DistLimit;
	 double Vmag;
	 double dmin;
	 bool ifsafe = true;
	 double ascend_pos;


	 // Establish distance threshold based on lander
	 // speed (we need more time to rectify direction
	 // at high speed)
	 Vmag=Velocity_X()*Velocity_X();
	 Vmag+=Velocity_Y()*Velocity_Y();

	 DistLimit=fmax(40, 40);
	 // If we're close to the landing platform, disable
	 // safety override (close to the landing platform
	 // the Control_Policy() should be trusted to
	 // safely land the craft)

	 // Determine the closest surfaces in the direction
	 // of motion. This is done by checking the sonar
	 // array in the quadrant corresponding to the
	 // ship's motion direction to find the entry
	 // with the smallest registered distance

	 // Horizontal direction.
	 dmin=1000000;

	//Rotate(deg + (angle_wrt_working_thru - (Angle() - 360)));
	
	// If vetical velocity is too fast
	if(fabs(Pos_Y - PLAT_Y) > 150  && fabs(Pos_X - PLAT_X) > 30)
	{
		 for (int i=0; i<36; i++)
		{
	   		if (SONAR_DIST[i] < DistLimit && SONAR_DIST[i] != -1)
			{
				ifsafe = false;
				ascend_pos = Pos_Y;
			}
		}
	}
	
	//printf("PosY = %f,   Angle = %f    \n ",  Position_Y(), Angle());
	//printf("Velocity X = %f,   Velocity Y = %f, error = %f   \n ",  Velocity_X(), Velocity_Y(), (Position_X() - PLAT_X));
	if (fabs(Pos_Y - PLAT_Y) < 30) {
		if_passed_s1 = true;
	}
	if (ifsafe) 
	{


		// error = Position_X - reference_s2
		double error = Pos_X - PLAT_X;
		// Set the user input to be the angle.
		double desired_angle = -0.08* error +  (-4)* Velocity_X() + angle_wrt_working_thru;

		// Covert any angle that is greater than 315 to negative
		//printf("Desired = %f, Error = %f", desired_angle, Position_X() - PLAT_X);

		if(Angle() > 270+ angle_wrt_working_thru)
		{
			Rotate((desired_angle - (Angle() - 360)));
		}
		else
		{
			Rotate((desired_angle - Angle()));
		}
		

		// If the difference between the spacecraft's horizontal position and the landing pad's
		// horizontal position is within 5 units, move to stage 3
		
		if(Velocity_Y() > 3|| Pos_Y < 30)
		{
			if (mode == 0) {
				Main_Thruster(0);
			} 	
			else if (mode == 1)
			{
				Left_Thruster(0);
			 }
			 else
			 {
				Right_Thruster(0);
			 }
			printf("bad");

		}
		else if(Velocity_Y() <-5)
		{
			if(fabs(error) > 30)
			{
				if (mode == 0) { 
					Main_Thruster(0.4);
				} 	                                    
				else if (mode == 1)
				{
					Left_Thruster(0.6);
				 }
				 else
				 {
					Right_Thruster(0.6);
				 }
			}
		}
		// If the error is within the range of 30 units, we will ascend it fast,
		// at the same time maintain the vertical velocity above 5. 
		if( fabs(error) < 30)
		{
			if(Velocity_Y() > -5 )
			{
				if (mode == 0) { 
					Main_Thruster(0.1);
				} 	                                    
				else if (mode == 1)
				{
					Left_Thruster(0.15);
				 }
				 else
				 {
					Right_Thruster(0.15);
				 }
			}
	
			else
			{
				if (mode == 0) { 
					Main_Thruster(0.3);
				} 	                                    
				else if (mode == 1)
				{
					Left_Thruster(0.4);
				 }
				 else
				 {
					Right_Thruster(0.4);
				 }
			}
		}

	}
	// If it is not safe, we need ascend veritcally the spacecrat up 75 units
	// with the respect to vertical position where initially detected the danger 
	else
	{
		ascend_straight();
		if(Pos_Y > ascend_pos + 50)
		{
			ifsafe = true;
		}
		printf("Aint safe");
	}


}

void ascend_straight()
{
	double error = 0;
	
	// Set the reference of the horizontal speed to be 0
	double V_x_ref = 0;
	// Create a variable for recording the x speed in the previous second
	double V_x_pre = 0;
	if(Angle() < 30 + angle_wrt_working_thru || Angle()> 330 + angle_wrt_working_thru)
	{	
		// Update the error
		error = Velocity_X() - V_x_ref;
		error_accumulator_s3 += error;
		//printf("Vert. Veloc. = %f, Distance From Platform: %f\n",  Velocity_Y(), Position_Y() - PLAT_Y);
		// For PID controller,  angle is the user input
		int Input_angle = -10*error + angle_wrt_working_thru;
		// Covert any angle that is greater than 315 to negative
		if(Angle() > 330 + angle_wrt_working_thru)
		{
			Rotate((Input_angle - (Angle() - 360)));
		}
		else
		{
			Rotate((Input_angle - Angle()));
		}
		
		if(Velocity_Y()  <  5)
		{
			if (mode == 0)
			{ 
			 Main_Thruster(0.8);
			} else if (mode == 1) {
				Left_Thruster(1.0);
			} else {
				Right_Thruster(1.0);
			}
		}
		else
		{
			Right_Thruster(0);
			Left_Thruster(0);
			Main_Thruster(0);
		}
	}
	else
	{
		  if (Angle()>1||Angle()>359)
		  {
		   if (Angle()>= 180) Rotate(360-Angle() + angle_wrt_working_thru);
		   else Rotate(-Angle() + angle_wrt_working_thru);
		  }
	}

}


void stage2() {

  Main_Thruster(0);
  Left_Thruster(0);
  Right_Thruster(0);

  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
  }
//printf("Vert. Veloc. = %f, Distance From Platform: %f\n",  Velocity_Y(), Position_Y() - PLAT_Y);
}

void set_angle_wrt_working_thruster()
{

	// If the working thruster is the main, angle_wrt_working_thru = 0;
	if(mode == 0)
	{
		angle_wrt_working_thru = 0;
	}
	// If the working thruster is the left,  angle_wrt_working_thru = -90;
	else if(mode == 1)
	{
		angle_wrt_working_thru = -90;
	}
	// If the working thruster is the right,  angle_wrt_working_thru = 90;
	else if(mode == 2)
	{
		angle_wrt_working_thru = 90;
	}

}
void Safety_Override(void)
{
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/
if(!if_safe)
{
 double DistLimit;
 double Vmag;
 double dmin;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=Velocity_X()*Velocity_X();
 Vmag+=Velocity_Y()*Velocity_Y();

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 if (Velocity_X()>0)
 {
  for (int i=5;i<14;i++)
  if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }

  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }

Main_Thruster(0.0);
 

 if (Angle() <2 || Angle()> 358 )   // Too close to a surface in the horizontal direction
 {
	if_safe = true;
	printf("safe");
 }


}

}



void check_pos_x_sensor() {

 if(fabs(Prev_X_Pos - Pos_X) >= 50) {
 	is_malfunc_pos_x = true;
 }

}

double Robust_Pos_X() {
 Pos_X = Prev_X_Pos + Prev_X_V /40;
printf("safe \n");
}



void check_pos_y_sensor() {

 if(fabs(Prev_Y_Pos - Pos_Y) >= 50) {
 	is_malfunc_pos_x = true;
 }

}

double Robust_Pos_Y() {
 Pos_Y = Prev_Y_Pos + Prev_Y_V /40;
printf("Unsafe\n");
}









