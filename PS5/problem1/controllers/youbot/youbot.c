  /*
 * File:          youbot.c
 * Date:          24th May 2011
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

#include <webots/robot.h>
#include <webots/camera.h>

#include "../../lib/base.h"
#include "../../lib/arm.h"
#include "../../lib/gripper.h"
#include "../../libobf/ik.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 32
#define LOW_BOUND 4


/*
 * Find the co-ordinates of the center of the green block.
 * Takes in the width and height of the camera image, as well
 * as the image array. 
 *
 **PLEASE NOTE *************************************
 * Returns the center co-ordinates x_coord
 * and y_coord, by reference. X is the distance along the horizontal
 * axis (the column number), and y is the distance along the vertical axis
 * (the row number). The top left is (0,0)
 ***************************************************
 *
 * To get the green, red, or blue values at a given co-ordinate
 * you should use 
 * (unsigned char)wb_camera_image_get_green(image,width,i,j)
 * (unsigned char)wb_camera_image_get_red(image,width,i,j)
 * (unsigned char)wb_camera_image_get_blue(image,width,i,j)
 *
 */
void get_green_block_center_coord(int width, int height, const unsigned char* image, int *x_coord, int *y_coord){
  //Put your return x-coordinate and y-coordinate in x_ret and y_ret
  int x_ret=0;
  int y_ret=0;
  
  // Your code here
  int r[width][height], g[width][height], b[width][height];
  int i, j;
  // Perceive RGB tunnel info by scanning each pixel in the rectangular range
  for (i = 0; i < width; i++) {
  	for (j = 0; j < height; j++) {
      r[i][j] = wb_camera_image_get_red(image, width, i, j);
      g[i][j] = wb_camera_image_get_green(image, width, i, j);
      b[i][j] = wb_camera_image_get_blue(image, width, i, j);
  	}
  }

  // Find center of boxes by summing up x & y coordinates, then find their mean, respectively
  int sx = 0, sy = 0, n = 0;
  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
    	// Condition to judge if the color can be significantly distinguished
      if (g[i][j] > LOW_BOUND * (r[i][j] + b[i][j]) && g[i][j] < r[i][j] * b[i][j]) {
        sx += i;
        sy += j;
        n++;
        // printf("[i j]=[%d %d], RGB=[%d %d %d]\n", i, j, r[i][j], g[i][j], b[i][j]);
      }
  	}
  }
  x_ret = round(sx / n);
  y_ret = round(sy / n);
  
  //"Return" the x_ret and y_ret
  *x_coord = x_ret;
  *y_coord = y_ret;
}

/*
 * Find the co-ordinates of the center of the red block.
 * Takes in the width and height of the camera image, as well
 * as the image array. 
 *
 * **PLEASE NOTE *************************************
 * Returns the center co-ordinates x_coord
 * and y_coord, by reference. X is the distance along the horizontal
 * axis (the column number), and y is the distance along the vertical axis
 * (the row number). The top left is (0,0)
 ***************************************************
 *
 * Returns the center co-ordinates x_coord
 * and y_coord, by reference.
 *
 * To get the green, red, or blue values at a given co-ordinate
 * you should use 
 * (unsigned char)wb_camera_image_get_green(image,width,i,j)
 * (unsigned char)wb_camera_image_get_red(image,width,i,j)
 * (unsigned char)wb_camera_image_get_blue(image,width,i,j)
 *
 */
void get_red_block_center_coord(int width, int height, const unsigned char* image, int *x_coord, int *y_coord){
  //Put your return x-coordinate and y-coordinate in x_ret and y_ret
  int x_ret=0;
  int y_ret=0;
  
  // Your code here
  int r[width][height], g[width][height], b[width][height];
  int i, j;
  // Perceive RGB tunnel info by scanning each pixel in the rectangular range
  for (i = 0; i < width; i++) {
  	for (j = 0; j < height; j++) {
      r[i][j] = wb_camera_image_get_red(image, width, i, j);
      g[i][j] = wb_camera_image_get_green(image, width, i, j);
      b[i][j] = wb_camera_image_get_blue(image, width, i, j);
  	}
  }

  // Find center of boxes by summing up x & y coordinates, then find their mean, respectively
  int sx = 0, sy = 0, n = 0;
  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
    	// Condition to judge if the color can be significantly distinguished
      if (r[i][j] > LOW_BOUND * (g[i][j] + b[i][j]) && r[i][j] < g[i][j] * b[i][j]) {
        sx += i;
        sy += j;
        n++;
        // printf("[i j]=[%d %d], RGB=[%d %d %d]\n", i, j, r[i][j], g[i][j], b[i][j]);
      }
    }
  }
  x_ret = round(sx / n);
  y_ret = round(sy / n);
  
  //"Return" the x_ret and y_ret
  *x_coord = x_ret;
  *y_coord = y_ret;
}

/*
 * Find the co-ordinates of the center of the blue block.
 * Takes in the width and height of the camera image, as well
 * as the image array.
 *
 **PLEASE NOTE *************************************
 * Returns the center co-ordinates x_coord
 * and y_coord, by reference. X is the distance along the horizontal
 * axis (the column number), and y is the distance along the vertical axis
 * (the row number). The top left is (0,0)
 ***************************************************
 *
 * Returns the center co-ordinates x_coord
 * and y_coord, by reference.
 *
 * To get the green, red, or blue values at a given co-ordinate
 * you should use 
 * (unsigned char)wb_camera_image_get_green(image,width,i,j)
 * (unsigned char)wb_camera_image_get_red(image,width,i,j)
 * (unsigned char)wb_camera_image_get_blue(image,width,i,j)
 *
 */
void get_blue_block_center_coord(int width, int height, const unsigned char* image, int *x_coord, int *y_coord){
  //Put your return x-coordinate and y-coordinate in x_ret and y_ret
  int x_ret=0;
  int y_ret=0;
  
  // Your code here
  int r[width][height], g[width][height], b[width][height];
  int i, j;
  // Perceive RGB tunnel info by scanning each pixel in the rectangular range
  for (i = 0; i < width; i++) {
  	for (j = 0; j < height; j++) {
      r[i][j] = wb_camera_image_get_red(image, width, i, j);
      g[i][j] = wb_camera_image_get_green(image, width, i, j);
      b[i][j] = wb_camera_image_get_blue(image, width, i, j);
  	}
  }

  // Find center of boxes by summing up x & y coordinates, then find their mean, respectively
  int sx = 0, sy = 0, n = 0;
  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
    	// Condition to judge if the color can be significantly distinguished
      if (b[i][j] > LOW_BOUND * (r[i][j] + g[i][j]) && b[i][j] < r[i][j] * g[i][j]) {
        sx += i;
        sy += j;
        n++;
        // printf("[i j]=[%d %d], RGB=[%d %d %d]\n", i, j, r[i][j], g[i][j], b[i][j]);
      }
  	}
  }
  x_ret = round(sx / n);
  y_ret = round(sy / n);
  
  //"Return" the x_ret and y_ret
  *x_coord = x_ret;
  *y_coord = y_ret;
}

/* Here you should put your code to sort the blocks according
 * to the instructions
 *
 * You may call other functions, but should at the very
 * least call:
 
 get_red_block_center_coord(...);
 get_blue_block_center_coord(...);
 get_red_block_center_coord(...);
 */
void sort_blocks(int width, int height, const unsigned char* image) {
 
  // obtain coordinates for the three blocks 
  int x_g=0, y_g=0, x_b=0, y_b=0, x_r=0, y_r=0;
  get_green_block_center_coord(width,height,image,&x_g,&y_g);
  get_blue_block_center_coord(width,height,image,&x_b,&y_b);
  get_red_block_center_coord(width,height,image,&x_r,&y_r);
  
  //Now that you know the coordinates for all three blocks, you can use the following functions to control the robot arm:
  //grab_ik(x_cood,y_coord); - makes the arm go to the specified position and grab the block it finds there
  //                           your coordinate estimates need to be within 5 pixels from the true coordinates
  //                           (i.e. the euclidean distance from the center of the object and the estimated center
  //                           of the object needs to be less than 5; if this is the case, the arm performs the movement
  //                           and the function returns 1; if it is not the case, the arm doesn't move at all and the 
  //                           function returns -1). NOTE: this check is performed by the code you are provided with, you 
  //                           do not need to do anything to check it yourself. You do, however, need to estimate the coordinates
  //                           well enough in order to make the arm move. 
  //drop_purple_bin(); - can only be called after the arm already has an object in its grasp; it moves the object above
  //                     the purple bin and drops it in
  //drop_yellow_bin(); - can only be called after the arm already has an object in its grasp; it moves the object above
  //                     the yellow bin and drops it in
  //reset_arm(); - resets arm to its default position
  
  grab_ik(x_g,y_g);
  drop_purple_bin();
  reset_arm();

  grab_ik(x_r,y_r);
  drop_purple_bin();
  reset_arm();

  grab_ik(x_b,y_b);
  drop_yellow_bin();
  reset_arm();

  printf("Green: (%d,%d)\n", x_g, y_g);
  printf("Red: (%d,%d)\n", x_r, y_r);
  printf("Blue: (%d,%d)\n", x_b, y_b);

}

int main(int argc, char **argv)
{
  WbDeviceTag camera;
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  int width=wb_camera_get_width(camera);
  int height=wb_camera_get_width(camera);
  while (wb_robot_step(TIME_STEP)==-1);
  const unsigned char *image = wb_camera_get_image(camera);
  
  sort_blocks(width, height, image);
  
  wb_robot_cleanup();
  
  return 0;
}
