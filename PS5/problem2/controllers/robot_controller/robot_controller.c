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
#include <webots/receiver.h>

#include "../../lib/base.h"
#include "../../lib/arm.h"
#include "../../lib/gripper.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  }
  while(start_time + sec > wb_robot_get_time());
}

void pickup_right() {
    gripper_release();
    arm_set_orientation(ARM_FRONT_RIGHT);
    arm_set_height(ARM_FRONT_FLOOR);
    passive_wait(5.0);
    gripper_grip();
    passive_wait(2.0);

}

void pickup_left() {
    gripper_release();
    arm_set_orientation(ARM_FRONT_LEFT);
    arm_set_height(ARM_FRONT_FLOOR);
    passive_wait(5.0);
    gripper_grip();
    passive_wait(2.0);

}

void pickup_center() {
    gripper_release();
    arm_set_orientation(ARM_FRONT);
    arm_set_height(ARM_FRONT_FLOOR);
    passive_wait(5.0);
    gripper_grip();
    passive_wait(2.0);

}

void drop_left() {
    arm_set_height(ARM_FRONT_PLATE);
    passive_wait(1.0);
    arm_set_orientation(ARM_LEFT);
    passive_wait(5.0);
    gripper_release();
    passive_wait(2.0);
}

void drop_right() {
    arm_set_height(ARM_FRONT_PLATE);
    passive_wait(1.0);
    arm_set_orientation(ARM_RIGHT);
    passive_wait(5.0);
    gripper_release();
    passive_wait(2.0);
}

void reset_arm() {
    arm_reset();
    passive_wait(6.0);
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
    
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
  arm_init();
  base_init();
  gripper_init();
  passive_wait(2.0);
           
  do {
  
    wb_receiver_enable(receiver, TIME_STEP);
    
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *message = (char *) wb_receiver_get_data(receiver);
      int msg;
      memcpy(&msg,message,sizeof(int));   

      switch(msg){
        case 0: 
        break;
        case 1: reset_arm(); pickup_left(); drop_right();            
        break;
        case 2: reset_arm(); pickup_left(); drop_left();
        break;
        case 3: reset_arm(); pickup_center(); drop_right();
        break;
        case 4: reset_arm(); pickup_center(); drop_left();
        break;
        case 5: reset_arm(); pickup_right(); drop_right();
        break;
        case 6: reset_arm(); pickup_right(); drop_left();
        break;
        case 10: reset_arm(); 
        break;
      }

      wb_receiver_next_packet(receiver);
    }
    
  } while (wb_robot_step(TIME_STEP) != -1);
  
  /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}
