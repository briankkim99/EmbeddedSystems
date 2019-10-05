// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#include "math.h"
#include <signal.h>

static int running = 1;
static bool is_scanning_intersection = false;
static bool resetting = false;
static int colorSample[3];
void exitHandler(int sig) {
	running = 0;
	BT_all_stop(1);
}

int main(int argc, char *argv[])
{
 char test_msg[8]={0x06,0x00,0x2A,0x00,0x00,0x00,0x00,0x01};
 char reply[1024];
 int tone_data[50][3];


 // Reset tone data information
 for (int i=0;i<50; i++) 
 {
   tone_data[i][0]=-1;
   tone_data[i][1]=-1;
   tone_data[i][2]=-1;
 }
 /*
 tone_data[0][0]=262;
 tone_data[0][1]=250;
 tone_data[0][2]=1;

 tone_data[1][0]=330;
 tone_data[1][1]=250;
 tone_data[1][2]=25;
 tone_data[2][0]=392;
 tone_data[2][1]=250;
 tone_data[2][2]=50;
 tone_data[3][0]=523;
 tone_data[3][1]=250;
 tone_data[3][2]=63;
*/

 memset(&reply[0],0,1024);
 
 //just uncomment your bot's hex key to compile for your bot, and comment the other ones out.
 #ifndef HEXKEY
 	#define HEXKEY "00:16:53:56:50:79"	// <--- SET UP YOUR EV3's HEX ID here
 #endif	

 BT_open(HEXKEY);

 // name must not contain spaces or special characters
 // max name length is 12 characters
 //BT_setEV3name("WALL-E");

 //BT_play_tone_sequence(tone_data);

// int color = BT_read_colour_sensor_RGB(PORT_1);


	int RGBSample[10][3];
	int numberOfSamples = 0;
	int oldColor = 0;
	bool yellow_read = false;
	signal(SIGINT, exitHandler);
	int colorswitch[3] = {-1, -1, -1};
	bool moving = false;

	while(running)
	{ 
		
		/************************************
		 Using sensors and collecting samples
		*************************************/

		int color_array[4];
		int newColor = GetColor();


		/************************************
		 Using sensors and collecting samples
		*************************************/

		/**********************************
		 Default mode, continue along road
		***********************************/

		//PID Controller for staying on road

		/**********************************
		 Default mode, continue along road
		***********************************/

		/***************
		 Changing Modes
		****************/

		/*
		if (newColor == 4) is_scanning_intersection = true;
		if(is_scanning_intersection) {
			//intersection_stage(color_array);
		} else {

			BT_turn(MOTOR_B, 5, MOTOR_C, -5);
		}
		*/

		BT_turn(MOTOR_B, 10, MOTOR_C, 0);
		printf("%d\n", BT_read_gyro_sensor(PORT_2));
		/***************
		 Changing Modes
		****************/

}

BT_motor_port_stop(MOTOR_C|MOTOR_B, 1);


}

// Stage 1:
void intersection_stage(int array[]) {
	
	int power[] = {0, 0};
	for (int i = 0; i < 4; i++) {

		if (i == 0) {
			power[0] = 0;
			power[1] = 10;
		} else if (i == 1) {
			power[0] = 10;
			power[1] = 0;
		} else if (i == 2) {
			power[0] = -10;
			power[1] = 0;
		} else { 
			power[0] = 0;
			power[1] = -10;
		}

		// Continue to turn while a new color isn't being read
		int color = GetColor();
		while (color == 4 || color == 1) {
			BT_turn(MOTOR_B, power[0], MOTOR_C, power[1]);

			color = GetColor();

			if(running == 0) {
				break;
			}
		}

		BT_turn(MOTOR_B, 0, MOTOR_C, 0);

		array[i] = color;
		/*
		while (color != 4) {
			BT_turn(MOTOR_B, -1*power[0], MOTOR_C, -1*power[1]);
			color = BT_read_colour_sensor(PORT_1);
			if(running == 0) {
				break;
			}
		}
		
		BT_turn(MOTOR_B, 0, MOTOR_C, 0);
		*/
	is_scanning_intersection = false;
	}
}

int GetColor() {

	int RGB[3];
	int newColor;
	int a = BT_read_colour_sensor_RGB(PORT_1, RGB);
	int maxval = 255;
	for (int i=0; i<3; i++) {
		if(RGB[i] > maxval) maxval = RGB[i];
	}
	if (maxval > 255) {
		for (int k=0; k<3; k++) {
			RGB[k] = (int) ((double) 255/maxval * RGB[k]);
		}
	}

	printf("%d %d %d \n", RGB[0], RGB[1], RGB[2]);

}



