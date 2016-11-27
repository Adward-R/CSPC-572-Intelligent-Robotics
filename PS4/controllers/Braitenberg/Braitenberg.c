/*
 * File:          Braitenberg.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/receiver.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif
#ifndef min
	#define min( a, b ) ( ((a) > (b)) ? (b) : (a) )
#endif



/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
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
  
  //Coefficients for the values going from left sensor to right 
  //motor
  float lr_a = 0.5;
  float lr_b = 0.5;
  float lr_c = 0.5; 
  
  //Coefficients for the values going from right sensor to left 
  //motor
  float rl_a = 1;
  float rl_b = 1;
  float rl_c = 1;
  
  // initialize devices
  int i;
  WbDeviceTag ps[2];
  char ps_names[2][4] = {
    "ls0", "ls1"
  };
  
  for (i=0; i<2; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_light_sensor_enable(ps[i], TIME_STEP);
  } 
  do {
    wb_receiver_enable(receiver, TIME_STEP); 
    //Try to get data from the GA
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *message = wb_receiver_get_data(receiver);
      float genes[6];
      memcpy(genes,message,sizeof(genes));   

      /*
      printf("received: %f,%f,%f,%f,%f,%f \n",genes[0],
            genes[1],genes[2],genes[3],genes[4],genes[5]);
      */

      //Coefficients for the values going from left sensor to right 
      //motor
      lr_a = genes[0];
      lr_b = genes[1];
      lr_c = genes[2]; 
      
      //Coefficients for the values going from right sensor to left 
      //motor
      rl_a = genes[3];
      rl_b = genes[4];
      rl_c = genes[5];
      

      wb_receiver_next_packet(receiver);
    }
    
    
    /* 
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(receiver);
     */
     
    float left_sensor = wb_light_sensor_get_value(ps[0]);
    float right_sensor = wb_light_sensor_get_value(ps[1]);


    
    /* Process sensor data here */
  
    float right_speed = lr_a*pow(left_sensor,2) + \
                        lr_b*left_sensor + \
                        lr_c;
    float left_speed = rl_a*pow(right_sensor,2) + \
                        rl_b*right_sensor + \
                        rl_c;
                        
    //printf("left-speed %f \n",left_speed);
    //printf("right-speed %f \n",right_speed);                        
                        
    right_speed = min(right_speed,100.0);
    left_speed = min(left_speed,100.0);
    
    right_speed = max(right_speed,0.0);
    left_speed = max(left_speed,0.0);

    
    wb_differential_wheels_set_speed(left_speed, right_speed);

    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
  } while (wb_robot_step(TIME_STEP) != -1);
  
  /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}
