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
static double Pos_X;
static double Pos_Y;
static double Vel_X;
static double Vel_Y;
static int t = 0;
static double power;
static double Rotate_angle;
static double angle;
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

  // When the program firs tstarts, estimate the positions and velocity by sample
  // the position and velocity
	if( t == 0)
	{
		Pos_X = Position_X();
		Pos_Y = Position_Y();
		Vel_X = Robust_Vel_X();
		Vel_Y =	Robust_Vel_Y();
		angle = Angle();
	}

	 // If the main thruster works, we change mode to be 0.
	 if(MT_OK)
	 {
	 	mode = 0;
		Right_Thruster(0.0);
		Left_Thruster(0.0);
	 }
	 // Otherwise, if the left thruster is working, we change the mode to be 1.
	 else if(LT_OK)
	 {
		Right_Thruster(0.0);
		Main_Thruster(0.0);
	 	mode = 1;
	 }
	 // Otherwise, the mode will be changed to be 2.
	 else if(RT_OK)
	 {
		Left_Thruster(0.0);
		Main_Thruster(0.0);
	 	mode = 2;
	 }
	 
	// Set the angle in respect to the thruster in use
	set_angle_wrt_working_thruster();

	// While the spacecraft has not begun the landing stage, continue to travel towards it
	if(!if_passed_s1)
	{
		stage1();
	}
	else
  // Start the landing stage
	{
		stage2();
	}
	// Update the variables
	angle = Robust_Angle();
	Vel_X = Robust_Vel_X();
	Vel_Y = Velocity_Y();
	Pos_X = Robust_Pos_X();
	Pos_Y = Robust_Pos_Y();
	t++;

}


// Function that handles the movement to the platform.
// This uses a PID controller to control its velocity and position to an ideal location
// before landing.
void stage1()
{	


	 double DistLimit;
	 double Vmag;
	 double dmin;
	 bool ifsafe = true;
	 double ascend_pos;
   DistLimit = 40;
	
	// While the spacecraft is not close to landing, use the sonar to check for potential collisions
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
	
	// If the spacecraft is close to the platform vertically, move to the landing stage
	if (fabs(Pos_Y - PLAT_Y) < 30){
		if_passed_s1 = true;
	}
	// While the spacecraft is not in danger of crashing, adjust the angle and velocity
	if (ifsafe) 
	{
		Thruster_and_Angle_Control();
	}
	// If it is not safe, we need ascend vertically to move away from the danger
	else
	{
		ascend_straight();
		if(Pos_Y > ascend_pos + 40) ifsafe = true;
	}

}

// Function that handles terrain avoidance using a PI controller
void ascend_straight()
{
	double error = 0;
	
	// Set the reference of the horizontal speed to be 0
	double V_x_ref = 0;
	// Create a variable for recording the x speed in the previous second
	double V_x_pre = 0;
	if(angle < 30 + angle_wrt_working_thru || angle> 330 + angle_wrt_working_thru)
	{	
		// Update the error
		error = Vel_X - V_x_ref;
		error_accumulator_s3 += error;
		// For PID controller,  angle is the user input
		int Input_angle = -8*error + angle_wrt_working_thru;
		// Covert any angle that is greater than 315 to negative
		if(angle > 330 + angle_wrt_working_thru)
		{
			Rotate_angle = Input_angle - (angle - 360);
			Rotate(Rotate_angle);
		}
		else
		{
			Rotate_angle = Input_angle - angle;
			Rotate(Rotate_angle);
		}
		
		// Limit the spacecraft's vertical velocity
		if(Velocity_Y()  <  5)
		{

			power = 0.7;
			Apply_Thruster_Power();
		}
		else
		{
			power = 0;
			Apply_Thruster_Power();
		}
	}
	else
	{
	      // Rotate c.c.w or c.w based on the initial angle and the desired angle
		   	if (angle>= 180 + angle_wrt_working_thru)
		   	{
				Rotate_angle = 360-angle + angle_wrt_working_thru;
				Rotate(Rotate_angle);
			}
		   	else
			{ 
				Rotate_angle = -angle + angle_wrt_working_thru;
				Rotate(Rotate_angle);
			}

	}

}

// Function that handles the landing of the spacecraft
void stage2() 
{
  // Turn off all thrusters
	Main_Thruster(0);
	Left_Thruster(0);
	Right_Thruster(0);
	// Rotate the spacecraft back to zero degrees
	if (angle>1||angle>359)
	{
  	if (angle>= 180 )
  	{
  		Rotate_angle = 360-angle;
  		Rotate(Rotate_angle);
  	}
  	else
  	{ 
  		Rotate_angle = -angle;
  		Rotate(Rotate_angle);
  	}
	}
}

void set_angle_wrt_working_thruster()
{

	// If the working thruster is the main, set the angle to 0
	if(mode == 0)
	{
		angle_wrt_working_thru = 0;
	}
	// If the working thruster is the left, the angle is now with respect to the left thruster
	else if(mode == 1)
	{
		angle_wrt_working_thru = -90;
	}
	// If the working thruster is the right,  the angle is now with respect to the right thruster
	else if(mode == 2)
	{
		angle_wrt_working_thru = 90;
	}

}
void Safety_Override(void)
{


}



double Robust_Pos_X() 
{
	if(t == 0)
	{
	  // Sample 1000 signals in the beginning from the sensor and find the average
		double Pos_X_Sum = 0;
		for(int i = 0; i < 1000; i++)
		{
			Pos_X_Sum += Position_X();
		}
		return Pos_X_Sum / 1000;
	}
	else
	// Return the estimated position
	{
		return Pos_X + Vel_X/40;
	}

}

double Robust_Vel_X()
{
	
	if(t == 0)
	{	
	  // Sample 1000 signals in the beginning from the sensor and find the average
		double Vel_X_Sum = 0;
		for(int i = 0; i < 1000; i++)
		{
			Vel_X_Sum += Velocity_X(); 
		}
		// Return the estimated velocity
		return Vel_X_Sum / 1000;
	}
	else
	{
    // Calculate the horizontal velocity generated by the thruster and the angle at which it is accelerating
		return Vel_X + (power * 35 * sin((angle - angle_wrt_working_thru )* M_PI / 180)) * 0.005013;
		

	}
}

double Robust_Vel_Y()
{
	
	if(t == 0)
	{	
	  // Sample 1000 signals in the beginning from the sensor and find the average
		double Vel_Y_Sum = 0;
		for(int i = 0; i < 1000; i++)
		{
			Vel_Y_Sum += Velocity_Y(); 
		}
		// Return the estimated velocity
		return Vel_Y_Sum / 1000;
	}
	else
	{
    // Calculate the vertical velocity generated by the thruster and the angle at which it is accelerating,
    // as well as factoring in the gravitational accleration
		return Vel_Y + (power * 35 * cos((angle - angle_wrt_working_thru ) * M_PI / 180)) * 0.00514 - 8.87 * 0.977 * 0.005;
		

	}
}


double Robust_Pos_Y() 
{
	if(t == 0)
	{
	  // Sample 1000 signals in the beginning from the sensor and find the average
		double Pos_Y_Sum = 0;
		for(int i = 0; i < 1000; i++)
		{
			Pos_Y_Sum += Position_Y();
		}

		return Pos_Y_Sum / 1000;
	}
	else
	{
	// Return the estimated position
	return Pos_Y = Pos_Y - Vel_Y/40;
	}
}

double Robust_Angle()
{
	double angle_sum = 0;
  // Sample 1000 signals in the beginning from the sensor and find the average
	for(int i = 0; i < 1000; i++)
	{
		angle_sum += Angle();
	}
	// Return the average angle, assuming noise is zero mean
	return angle_sum / 1000;
}
void Thruster_and_Angle_Control()
{
	
	// error = Position_X - reference_s2
	double error = Pos_X - PLAT_X;
	// Set the user input to be the angle.
	double desired_angle = -0.08* error +  (-4)* Vel_X + angle_wrt_working_thru;
	printf("     desired angle = %f \n", desired_angle ); 
	// Covert any angle that is greater than 270 to negative


	if(angle > 270 + angle_wrt_working_thru)
	{
		Rotate(desired_angle - (angle - 360));
	}
	else
	{
		Rotate((desired_angle - Angle()));
	}

	// Keep the vertical velocity below 3 and turn off all thruster when
	// spacecraft is too close to the ceiling
	if(Velocity_Y() > 3|| Pos_Y < 30)
	{
		power = 0;
		Apply_Thruster_Power();

	}
	// If the spacecraft's x position is close to the platform, slow down if it's past the limit
	else if(Velocity_Y() <-5)
	{
		if(fabs(error) > 30)
		{
			power = 0.4;
			Apply_Thruster_Power();
		}
	}
	// If the spacecraft's x position is close to the platform, start a descent at a safe velocity
	if( fabs(error) < 30)
	{
		if(Velocity_Y() > -5 )
		{
			power = 0.1;
			Apply_Thruster_Power();
		}

		else
		{
			power = 0.3;
			Apply_Thruster_Power();
		}
	}
}

// Depending on which thruster is in use, power it
void Apply_Thruster_Power()
{
			if (mode == 0) { 
				Main_Thruster(power);
			} 	                                    
			else if (mode == 1)
			{
				Left_Thruster(power * 7 /5 );
			 }
			 else
			 {
				Right_Thruster(power * 7 /5);
			 }
}

// UNFINISHED FUNCTION
// Function that utilizes the RangeDist() to scan the surroundings, allowing for accurate readings
// without relying on the sonar sensor.

/*

void Scan(void) {

	Main_Thruster(0);
	Left_Thruster(0);
	Right_Thruster(0);
	// Rotate so the laser finder is perpendicular to the ground
	



	// Then rotate the spacecraft 360 degrees, taking each 10 degrees as a distance		



	if (z % 3 == 0) {
		Rotate(9);	
	}


	
	div_t div_ang;
	div_ang = div (z, 3);
	ScanDist[div_ang.quot] = RangeDist();
	//printf("%f\n", RangeDist());
	if (z % 120 == 0) {
		scanning = false;

		t++;
		z = 0;
		printf("%f\n", Angle());
	}
	z++;

}
*/




