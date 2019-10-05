/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"
#include <signal.h>

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction
int RGB_Sample[2][3];
int oldColor;
static int inter_angle;
double belMinus[400][4];
int turnDir;
static int tone_data[50][3];

static int HSV_RED[3];
static int HSV_BLUE[3];
static int HSV_WHITE[3];
static int HSV_GREEN[3];
static int HSV_YELLOW[3];
static int HSV_BLACK[3];


void INThandler(int sig) {
 BT_all_stop(1);
 BT_close();
 exit(0);

 }

int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }


 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/
 

 /*******************
  Sensor Calibration
 *******************/

  // Read from HSV.txt file

  FILE *fp = fopen("HSV.txt", "r");
  
  char buf[100];
 
  int colourarr[6][3];

  int i = 0;
  int j = 0;
  char *tokens;
 while(fgets(buf, sizeof(buf), fp) != NULL) {
   tokens = strtok(buf, " ");
   while (tokens != NULL) {
    colourarr[i][j] = atoi(tokens);
    tokens = strtok(NULL, " ");
    j++;
   }
  j=0;
  i++;
 }

 memcpy(HSV_RED, colourarr[0], sizeof(HSV_RED));
 memcpy(HSV_BLUE, colourarr[1], sizeof(HSV_BLUE));
 memcpy(HSV_WHITE, colourarr[2], sizeof(HSV_WHITE));
 memcpy(HSV_GREEN, colourarr[3], sizeof(HSV_GREEN));
 memcpy(HSV_YELLOW, colourarr[4], sizeof(HSV_YELLOW));
 memcpy(HSV_BLACK, colourarr[5], sizeof(HSV_BLACK));
 fclose(fp);




 /*******************
  Sensor Calibration
 *******************/

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 fprintf(stderr,"All set, ready to go!\n");

 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }



/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.


 int is_localized = robot_localization(dest_x, dest_y);

 if (is_localized) {

 }


 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
 BT_close();

 free(map_image);
 exit(0);
}

// Gets the color and normalizes the value (RGB ranges from 1 - 255)
int get_color() {

 signal(SIGINT, INThandler);
 int RGB[3];

 int newColor;

 int a = BT_read_colour_sensor_RGB(PORT_1, RGB);

 int maxval = 255;
 bool Is_Valid_RBG = RGB_Checker(RGB);

 while (a == -1 || !Is_Valid_RBG)
 {
  BT_all_stop(1);
   a = BT_read_colour_sensor_RGB(PORT_1, RGB);
   Is_Valid_RBG = RGB_Checker(RGB);

 }
 
 for (int i=0; i<3; i++) {
  if(RGB[i] > maxval) maxval = RGB[i];
 }
 if (maxval > 255) {
  for (int k=0; k<3; k++) {
   RGB[k] = (int) ((double) 255/maxval * RGB[k]);
 }
 }


 float *h = (float*) malloc(sizeof(float));
 float *s = (float*) malloc(sizeof(float));
 float *v = (float*) malloc(sizeof(float));
 
 rgb_to_hsv(RGB[0], RGB[1], RGB[2], h, s, v);

 newColor = hsv_to_color(h, s, v);
 return newColor;
}


bool validate_color(int newcolor) {
 int newcolor_confirm;
 for(int i=0; i < 5; i++)
 {
  newcolor_confirm = get_color();
  if(newcolor != newcolor_confirm) return false;
 }
 return true;
}

void updateBelMinus(int turn) {
 ////////////////////////////////
 // turn = 0, no turn
 // turn = 1, turn right
 // turn = 2, turn left
 // turn = 3, turn 180
 //////////////////////////////

 // Loop over the intersections.
 for(int i = 0; i < sy * sx; i++) {
  // Loop over the directions of the current intersection.
   for(int j = 0; j < 4; j++) {
    // If the robot is facing up.
    if(j == 0) {
     // If the current position is at the bottom most row, 
     // the previous position after turning should be i and face down.
     if(i >= ((sy-1) * sx)) {
       if(turn == 0)
        belMinus[i][0] = 1 * beliefs[i][2];
       if(turn == 1)
        belMinus[i][0] = 1 * beliefs[i][1];
       if(turn == 2)
        belMinus[i][0] = 1 * beliefs[i][3];
       if(turn == 3)
        belMinus[i][0] = 1 * beliefs[i][0];
     }
     // Otherwise, privous position is 1 row down to the current position, and face up.
     else {
      if(turn == 0)
       belMinus[i][0] = 1 * beliefs[i + sx][0];
      if(turn == 1)
       belMinus[i][0] = 1 * beliefs[i + sx][3];
      if(turn == 2)
       belMinus[i][0] = 1 * beliefs[i + sx][1];
      if(turn == 3)
       belMinus[i][0] = 1 * beliefs[i + sx][2];
     }
    }
    // If the robot is facing right.
    if(j == 1) {
     // If the robot current pos is at the left most column,
     // the previous position should be i and face left.
     if((i % sx) == 0) {
      if(turn == 0)
       belMinus[i][1] = 1 * beliefs[i][3];
      if(turn == 1)
       belMinus[i][1] = 1 * beliefs[i][2];
      if(turn == 2)
	belMinus[i][1] = 1 * beliefs[i][0];
      if(turn == 3)
        belMinus[i][1] = 1 * beliefs[i][1];
     }
     // Otherwise, previous posiition is 1 column left to the current position, and face right.
     else {
      if(turn == 0)
       belMinus[i][1] = 1 * beliefs[i - 1][1];
      if(turn == 1)
       belMinus[i][1] = 1 * beliefs[i - 1][0];
      if(turn == 2)
        belMinus[i][1] = 1 * beliefs[i - 1][2];
      if(turn == 3)
        belMinus[i][1] = 1 * beliefs[i - 1][3];
     }
    }
    // If the robot is facing down.
    if(j == 2) {
     // If the robot current pos is at the top most row,
     // the previous position should be i and face up.
     if(i < 4) {
      if(turn == 0)
       belMinus[i][2] = 1 * beliefs[i][0];
      if(turn == 1)
       belMinus[i][2] = 1 * beliefs[i][3];
      if(turn == 2)
	belMinus[i][2] = 1 * beliefs[i][1];
      if(turn == 3)
       belMinus[i][2] = 1 * beliefs[i][2];
     }
     // Otherwise, the previous position is 1 row up to the current position, and face down.
     else {
      if(turn == 0)
       belMinus[i][2] = 1 * beliefs[i - sx][2]; 
      if(turn == 1)
       belMinus[i][2] = 1 * beliefs[i - sx][1];
      if(turn == 2)
       belMinus[i][2] = 1 * beliefs[i - sx][3];
      if(turn == 3)
       belMinus[i][2] = 1 * beliefs[i - sx][0];   
     }
    }
    // If the robot is facing left.
    if(j == 3) {
     // If the robot current pos is at the right most column,
     // the previous position should be i and face right.
     if((i % sx) == 2) {
      if(turn == 0)
       belMinus[i][3] = 1 * beliefs[i][1];
      if(turn == 1)
       belMinus[i][3] = 1 * beliefs[i][0];
      if(turn == 2)
       belMinus[i][3] = 1 * beliefs[i][2];
      if(turn == 3)
       belMinus[i][3] = 1 * beliefs[i][3];
     }
     // Otherwise, the previous position is 1 column right to the current position, and face left.
     else {
       if(turn == 0)
       	belMinus[i][3] = 1 * beliefs[i + 1][3];
       if(turn == 1)
	belMinus[i][3] = 1 * beliefs[i + 1][2];
       if(turn == 2)
	belMinus[i][3] = 1 * beliefs[i + 1][0];
       if(turn == 3)
        belMinus[i][3] = 1 * beliefs[i + 1][1];
     }
    }
   }
 }
}

// Cal the beliefs.
void calculateBeliefs(int colorInterReadings[4]) {
 double belSum = 0;
 // Loop over all the intersections
 for(int i = 0; i < sy * sx; i++) {
  // Loop over all the directions of the current intersection.
  for(int j = 0; j < 4; j++) {
    double prob = 1;
   // Compare the colorInterReadings with the colours around the current intersection from tl, tr, br to bl with the
   // respect to the current direction.
   for(int z = 0; z < 4; z++) {
    int currDirColor = map[i][((j + z) % 4)];
    if(colorInterReadings[z] == currDirColor) prob = prob * 0.7;
    else prob = prob * 0.1;
   }
   // Update believe at intersection i with direction j.
   beliefs[i][j] = prob * belMinus[i][j];
   belSum += prob * belMinus[i][j];
  }
 }

 // Normalize beliefs
 for(int i = 0; i < sy * sx; i++ ) {
  for(int j = 0; j < 4; j++) {
   beliefs[i][j] = beliefs[i][j] / belSum;
  }
 }
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  
  // if robot_x < target_x, turn to right
  if(robot_x < target_x) {
   if(direction == 0) {
    // turn right.
    turnDir = 1;
    printf("a");
   }
   if(direction == 1) {
    // Dont turn
    turnDir = 0;
        printf("b");
   }
   if(direction == 2) {
    // turn left
    turnDir = 2;
        printf("c");
   }
    if(direction == 3) {
    // turn 180
    turnDir = 3;
        printf("d");
   }
   return 1;
  }
  //robot_x > target_x , turn to left
  else if (robot_x > target_x) {
   if(direction == 0) {
    // turn left.
    turnDir = 2;
        printf("e");
   }
   if(direction == 1) {
    // turn 180 
    turnDir = 3; 
         printf("f"); 
   }
   if(direction == 2) {
    // turn right
    turnDir = 1;
        printf("g");
   }
   if(direction == 3) {
    // dont turn
    turnDir = 0;
       printf("h");
   }
    return 1;
  }
  // robot_y < target_y, turn down
  else if(robot_y < target_y) {
   if(direction == 0) {
    // turn 180
    turnDir = 3;
        printf("i");
   }
   if(direction == 1) {
   // turn right
   turnDir = 1;
        printf("j");
   }
   if(direction == 2) {
   // dont turn
   turnDir = 0;
       printf("k");
   }
   if(direction == 3){
   // turn left
   turnDir = 2;
        printf("l");
   }
   return 1;
  }
  // If robot_y > target_y, turn to up;
  else if(robot_y > target_y) {
   if(direction == 0) {
   // Dont turn
    turnDir = 0;
        printf("m");
   }
   if(direction == 1) {
   // turn left
    turnDir = 2;
        printf("n");
   }
   if(direction == 2) {
   // turn 180
    turnDir = 3;
         printf("o");
   }
    if(direction == 3)  {
   // turn right.
    turnDir = 1;
       printf("p");
   }
   return 1;
  }

  return(0);  
}
void findBelPosDir(int PosDir[2]) {
 PosDir[0] = 0;
 PosDir[1] = 0;
 double biggestBelieve = 0;
 for(int i = 0; i < sy* sx; i++) {
  for(int j = 0; j < 4; j++) {
   if(beliefs[i][j] > biggestBelieve) {
    biggestBelieve = beliefs[i][j];
    PosDir[0] = i;
    PosDir[1] = j;
   }
  }
 }
 fprintf(stderr, "BelPos = %d  BelDir = %d \n", PosDir[0], PosDir[1]);
}

void calculatePosDir (int *prevX, int *prevY, int*prevDir) {
 ////////////////////////////////////////////////////////////////////////////////////////
 // This function can only be called after the current position of the bot is confirmed.
 // By given the X, Y position and the direction from the last second combing with the turning direction,
 // we could predict the XY position and the direction at the present, and update them.
 ///////////////////////////////////////////////////////////////////////////////////////
 // Previously facing up
 if(*prevDir == 0) {
  // No turn, move upwards
   if(turnDir == 0) {
  *prevY = *prevY - 1;  
  }
  // Turn right, move right
  if(turnDir == 1) {
   *prevX = *prevX + 1;
   *prevDir = 1;
  }
  // Turn left, move left
  if(turnDir == 2) {
   *prevX = *prevX - 1;
   *prevDir = 3;
  }
  // Turn 180, move downwards
  if(turnDir == 3) {
   *prevY = *prevY + 1;
   *prevDir = 2;
  }
 }
 
 // Previously facing right
 else if(*prevDir == 1) {
  // No turn, move right.
  if(turnDir == 0) {
   *prevX = *prevX + 1;
  }
  // Turn right, move down
  if(turnDir == 1) {
   *prevY = *prevY + 1;
   *prevDir = 2; 
  }
  // Turn left, move up
  if(turnDir == 2) {
   *prevY = *prevY - 1;
   *prevDir = 0;
  }
  // Turn 180, move left
  if(turnDir == 3) {
   *prevX = *prevX - 1;
   *prevDir = 3;
  }
 }
 
 // Previously facing down
 else if(*prevDir == 2) {
  // No turn, move down
  if(turnDir == 0) {
   *prevY = *prevY + 1;
  }
  // Turn right, move left
  if(turnDir == 1) {
   *prevX = *prevX -1;
   *prevDir = 3;
  }
  // Turn left, move right
  if(turnDir == 2) {
   *prevX = *prevX + 1;
   *prevDir = 1;
  }
  // Turn 180, move up
  if(turnDir == 3) {
   *prevY = *prevY - 1;
   *prevDir = 0;
  }
 }
 
 // Previously facing left
 else if(*prevDir == 3) {
  // No turn, move left
  if(turnDir == 0) {
   *prevX = *prevX - 1;
  }
  // Turn right, move up
  if(turnDir == 1) {
   *prevY = *prevY -1;
   *prevDir = 0;
  }
  // Turn left, move down
  if(turnDir == 2) {
   *prevY = *prevY + 1;
   *prevDir = 2;
  }
  // Turn 180, move right
  if(turnDir == 3) {
   *prevX = *prevX + 1;
   *prevDir = 1;
  } 
 } 
}

bool RGB_Checker(int RGB[3]) {
 //fprintf(stderr, "%d ", RGB[0]);
 //fprintf(stderr, "%d ", RGB[1]);
 //fprintf(stderr, "%d \n", RGB[2]);
// If the RGB is identical to the previous two samples, or negative RGB,
// Stop the car and return flase.
// Otherwise, return true;

 BT_all_stop(1);
 for(int i = 1; i >= 0; i--) {
   for(int j = 0; j <3; j++) {
   if(RGB[j] != RGB_Sample[i][j]){
      RGB_Sample[0][0] = RGB_Sample[1][0];
     RGB_Sample[0][1] =   RGB_Sample[1][1];
     RGB_Sample[0][2] =  RGB_Sample[1][2];
     RGB_Sample[1][0] =  RGB[0];
     RGB_Sample[1][1] =  RGB[1];
     RGB_Sample[1][2] =  RGB[2];
     return true; 
   } 
   if(RGB[j] < 0) {
    BT_all_stop(1);
    return false;
   }
  }
 }
 
 return false;
 
}
void rgb_to_hsv(float r, float g, float b, float *h, float *s, float *v) {
 float fr, fg, fb;
 fr = r/255;
 fg = g/255;
 fb = b/255;
 float cmax = fmax(fmax(fr, fg), fb);
 float cmin = fmin(fmin(fr, fg), fb);
 float delta = cmax - cmin;

 if (delta > 0) {
  if (cmax == fr) {
   *h = 60 * fmod(((fg - fb)/ delta), 6);
  } else if (cmax == fg) {
   *h = 60 * (((fb - fr)/ delta) + 2);
  } else if (cmax == fb) {
   *h = 60 * (((fr - fg)/ delta) + 4);
  }

  if (cmax > 0) {
   *s = delta/cmax * 100;
  } else {
   *s = 0;
  }

  *v = cmax * 100;
 } else {
  *h = 0;
  *s = 0;
  *v = cmax * 100;
 }

 if (*h < 0) {
  *h = 360 + *h;
 }

 
}

int hsv_to_color(float *h, float *s, float *v) {
  int color = -1;
  if(*s < 50 ) {
    if(*v > 50 ) color = 6;
    else color = 1;

  }
  else {

    if(*h < (HSV_RED[0] + HSV_YELLOW[0])/2) color = 5;
    else if(*h < (HSV_RED[0] + HSV_GREEN[0]/2)) color = 4;
    else if(*h < (HSV_BLACK[0] + HSV_BLUE[0]/2)) color = 3;
    else if(*h < HSV_WHITE[0]) color = 2;
 
  }


  return color;
}

int update_color() {

  int new_color = get_color();

  if(new_color != oldColor) {
  	 BT_all_stop(1);
    // Call the validate method;
    // if it is not validated ,we use the old_color;
    if(!validate_color(new_color)) {
      return oldColor;
    }

  }

  
  oldColor = new_color;
  //fprintf(stderr, "old = %d,  new = %d\n", oldColor, new_color);
  return new_color;
  

}

int find_street(void)   
{
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective a reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */   
  return(0);
}

int drive_along_street(void)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   

  
  // Maintain a(n) 0/360, 90, 180, 270 angle so that it drives along the road

  // right now, go straight (0 degrees)
  int des_angle = inter_angle;
  int curr_angle = get_angle();
  printf("%d\n", curr_angle);

  
  // if the angle is 0 degrees, keep going straight and power wheels equally
  if (curr_angle < des_angle + 2 && curr_angle > des_angle - 2) {
   BT_turn(MOTOR_B, 10, MOTOR_C, 10);
  }
  // if the angle is less than 0 degrees, then decrease the power in the right wheel so
  // the robot will turn right
  else if (curr_angle <= des_angle ) {
   BT_turn(MOTOR_B, 10, MOTOR_C, 7);
  }
  // if the angle is more than 0 degrees, then decrease the power in the left wheel so
  // the robot will turn left
  else if (curr_angle >= des_angle) {
   BT_turn(MOTOR_B, 7, MOTOR_C, 10);
  }



  return (0);
}

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
 signal(SIGINT, INThandler);
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)

 int power[] = {0, 0};
 for (int i = 0; i < 4; i++) {
  if (i == 0) {
   power[0] = 0;
   power[1] = 10;
  } else if (i == 1) {
   power[0] = 10;
   power[1] = 0;
  } else if (i == 2) {
   power[0] = 0;
   power[1] = -10;
  } else { 
   power[0] = -10;
   power[1] = 0;
  }

  // Continue to turn while a new color isn't being read
  int color = update_color();
  while (color == 4 || color == 1) {
   printf("%d %d\n", inter_angle, get_angle());
   if (power[0] == 0) BT_motor_port_stop(MOTOR_B, 1);
   else BT_motor_port_start(MOTOR_B, power[0]);
   if (power[1] == 0) BT_motor_port_stop(MOTOR_C, 1);
   else BT_motor_port_start(MOTOR_C, power[1]);
   color =  update_color();
  }

  if (i == 0) *tl = color;
  else if (i == 1) *tr = color;
  else if (i == 2) *br = color;
  else if (i == 3) *bl = color;

  while (color != 4 || fabs(get_angle() - inter_angle) > 5) {
   color =  update_color();

   if (power[0] == 0) BT_motor_port_stop(MOTOR_B, 1);
   else BT_motor_port_start(MOTOR_B, -power[0]);
   if (power[1] == 0) BT_motor_port_stop(MOTOR_C, 1);
   else BT_motor_port_start(MOTOR_C, -power[1]);
  }
  }

  BT_all_stop(1);
 
 return(0);
 
}

int get_angle() {

 signal(SIGINT, INThandler);

 // sample angle 3 times
 int angle[5] = {0, 0, 0, 0, 0};


 for (int i=0; i<5; i++) {
  angle[i] = BT_read_gyro_sensor(PORT_2); 
 }
 
 int sum = 0;
 for (int k=0; k<5; k++) {
  sum += angle[k];
 }
 
 return (int) (sum/5);
}


void turn_around() {

 int des_angle = get_angle() + 180;
 int curr_angle = get_angle();
 int error = des_angle - curr_angle;
 int color;
 while (fabs(error) > 3) { 
  color = update_color();
  BT_turn(MOTOR_B, 10, MOTOR_C,  -10);
  curr_angle = get_angle();
  error = des_angle - curr_angle;
  printf("%d %d\n", des_angle, curr_angle);
  
 }
}

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */


  // If turn_direction = 0, set des_angle to be angle + 90
  // else des_angle = angle - 90

  int color = update_color();

  int angle = get_angle();
  int des_angle;
  if (turn_direction == 0) des_angle = angle + 90;
  else des_angle = angle + 270;
  int power[2];

  int error = des_angle - angle;
  while (fabs(angle - des_angle) > 10 || color != 4) {

   
   printf("%d\n", des_angle - angle);
   angle = get_angle();
   
   
   if (color == 1 || color == 4) {
    if (turn_direction == 1) {
     power[1] = 10 + (int) 10/error; 
     power[0] = -8 + (int) 110/error;
    } else {
     power[0] = 10 + (int) 10/error; 
     power[1] = -8 + (int) 110/error;
    }
   }
   else if (error > 60) {
    if(turn_direction == 1) {
     power[0] = -10;
     power[1] = 0;
    } else { 
    power[0] = 0;
    power[1] = -10;
    }
   }
   else if (error < 60) { 
    power[0] = 10;
    power[1] = 10;
   }
   BT_turn(MOTOR_B, power[0], MOTOR_C, power[1]);

   error = des_angle-angle;
   color = update_color();
  }

  while (fabs(angle - des_angle) > 2) {
   angle = get_angle();
   printf("%d\n", des_angle - angle);
   if (angle - des_angle > 0) BT_turn(MOTOR_B, -5, MOTOR_C, 5);
   else BT_turn(MOTOR_B, 5, MOTOR_C, -5);

  }
  inter_angle += 90;
  
  return(0);
}

int robot_localization(int dest_x, int dest_y)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Return an invalid location/direction and notify that localization was unsuccessful (you will delete this and replace it
 // with your code).

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }



 // Get 2 rgb_samples at the beginning.
 for(int i = 0; i++; i< 2)
 {
    BT_read_colour_sensor_RGB(PORT_1, RGB_Sample[i]); 
    oldColor = get_color();
 }


 int *tl = (int*) malloc(sizeof(int));
 int *tr = (int*) malloc(sizeof(int));
 int *bl = (int*) malloc(sizeof(int));
 int *br = (int*) malloc(sizeof(int));

 int belPosDir[2] = {0, 0};
 int turnDir = 0;
 int colorReading[4] = {0,0,0,0};
 int go =0;
 int bot_y = 0;
 int bot_x = 0;
 int dir = 0;

 //turn_around();
 int notFound = 1;


 while (notFound) {

  // Update color

  int color1 = update_color();

  // Move along the road
  while(color1 != 4)
  {
   drive_along_street();
   color1 = update_color();
  }
  color1 = update_color();
  if (color1 == 4) {
   // Start intersection scan if less than 80% sure of the current location
    if(beliefs[belPosDir[0]][belPosDir[1]] < 0.8) {
     updateBelMinus(turnDir);
     inter_angle = get_angle();
     int a = scan_intersection(tl, tr, br, bl);
     fprintf(stderr,"%d ", *tl);
     fprintf(stderr,"%d ", *tr);
     fprintf(stderr,"%d ", *br);
     fprintf(stderr,"%d ", *bl);
     colorReading[0] = *tl;
     colorReading[1] = *tr;
     colorReading[2] = *br;
     colorReading[3] = *bl;
     calculateBeliefs(colorReading);

     findBelPosDir(belPosDir);
    
     if(beliefs[belPosDir[0]][belPosDir[1]] > 0.5) {
      bot_x = belPosDir[0] % sx;
      bot_y = belPosDir[0] / sx;
      dir = belPosDir[1];
      go_to_target(bot_x, bot_y, dir, dest_x, dest_y);
     } else turnDir = 1;
    }
   } else {
    calculatePosDir(&bot_x, &bot_y, &dir);
    printf(" bot_x = %d,  bot_y = %d, dir = %d \n", bot_x, bot_y, dir);
    notFound = go_to_target(bot_x, bot_y, dir, dest_x, dest_y);
   }
   
   if(turnDir == 1) {
    turn_at_intersection(0);
   } else if (turnDir == 2) {
    turn_at_intersection(1);
   } else if (turnDir == 3) {
    turn_around();
   }

   while(color1 != 4) {
    drive_along_street();
    color1 = update_color();
   }

 } 


 inter_angle = get_angle();
 int a = scan_intersection(tl, tr, br, bl);
 fprintf(stderr,"%d ", *tl);
 fprintf(stderr,"%d ", *tr);
 fprintf(stderr,"%d ", *br);
 fprintf(stderr,"%d ", *bl);
 fprintf(stderr,"\n angle = %d ", BT_read_gyro_sensor(PORT_2));

 signal(SIGINT, INThandler);


 free(tl);
 free(tr);
 free(br);
 free(bl);
 BT_all_stop(1);


 if (notFound == 0) return 1;
 else return 0;
}


void calibrate_sensor(void)
{
 /*
  * This function is called when the program is started with -1  -1 for the target location. 
  *
  * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
  * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
  * level.
  * 
  * The principle is - Your code should allow you to sample the different colours in the map, and store representative
  * values that will help you figure out what colours the sensor is reading given the current conditions.
  * 
  * Inputs - None
  * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
  * 
  * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
  */   


  BT_open(HEXKEY);
  char color[10];
  int RGB[3];
  int newColor;
  int maxval = 255;
  float avg_h;
  float avg_s;
  float avg_v;
  int line = 0;
  FILE *fp = fopen("HSV.txt", "w");

  // FOLLOW THE ORDER OF COLOURS
  // RED, BLUE, WHITE, GREEN, BLACK, YELLOW

  for (int i=0;i<6;i++) {

   fgets(color, 10, stdin);
   avg_h = 0;
   avg_s = 0;
   avg_v = 0;
   for (int k=0;k<10;k++) {
        RGB[0] = 0;
	RGB[1] = 0;
	RGB[2] = 0;

	maxval = 255;
	int a = BT_read_colour_sensor_RGB(PORT_1, RGB);
	for (int a=0; a<3; a++)
	 if(RGB[a] > maxval) maxval = RGB[a];
	if (maxval > 255)
	 for (int b=0; b<3; b++) 
	  RGB[b] = (int) ((double) 255/maxval * RGB[b]);

	float *h = (float*) malloc(sizeof(float));
	float *s = (float*) malloc(sizeof(float));
	float *v = (float*) malloc(sizeof(float));

	rgb_to_hsv(RGB[0], RGB[1], RGB[2], h, s, v);

	printf("%0.f %0.f %0.f\n", *h, *s, *v);

	avg_h += *h;
	avg_s += *s;
	avg_v += *v;
    }

    avg_h = avg_h/10;
    avg_s = avg_s/10;
    avg_v = avg_v/10;

    fprintf (fp, "%0.f %0.f %0.f\n", avg_h, avg_s, avg_v);

  }

  fclose(fp);

  BT_close();
  /************************************************************************************************************************
   *   OPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/

 
}



int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}

