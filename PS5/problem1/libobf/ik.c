#include "../lib/arm.h"
#include "../lib/gripper.h"
#include "ik.h"

#include <webots/robot.h>
#include <webots/motor.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 32

static void AbAfkObiruP() {if (wb_robot_step(TIME_STEP) == -1) {wb_robot_cleanup();exit(EXIT_SUCCESS);}}static void cnjkwnfowDFzcxljvwjeZc(double sec) {double start_time = wb_robot_get_time();do {AbAfkObiruP();}while(start_time + sec > wb_robot_get_time());}void BDFBxclkvjeoAvaRVRPOVjQXXV() {arm_set_orientation(ARM_FRONT_RIGHT);}void bpisuhpibSBPIUEHRVBvdjGEGJ() {arm_set_orientation(ARM_FRONT_LEFT);}void KJHIUNBJHYnbvcxewasdfgh() {arm_set_orientation(ARM_FRONT);}void dbtyvucioxybtnrmJVHBGNFTREIOW(){gripper_grip();}void bngjfFKFKOQPQPSLCavosoaowej(){gripper_release();}void VBMFGJKQPOZVCNKsdljfkVJIP() {bngjfFKFKOQPQPSLCavosoaowej();BDFBxclkvjeoAvaRVRPOVjQXXV();arm_set_height(ARM_FRONT_FLOOR);cnjkwnfowDFzcxljvwjeZc(5.0);dbtyvucioxybtnrmJVHBGNFTREIOW();cnjkwnfowDFzcxljvwjeZc(2.0);}void BEOJWFNKJVSBKNwefjovnkjAVLNJ() {bngjfFKFKOQPQPSLCavosoaowej();bpisuhpibSBPIUEHRVBvdjGEGJ();arm_set_height(ARM_FRONT_FLOOR);cnjkwnfowDFzcxljvwjeZc(5.0);dbtyvucioxybtnrmJVHBGNFTREIOW();cnjkwnfowDFzcxljvwjeZc(2.0);}void ogwrijdlnvjkWEPFVZCNM() {bngjfFKFKOQPQPSLCavosoaowej();KJHIUNBJHYnbvcxewasdfgh();arm_set_height(ARM_FRONT_FLOOR);cnjkwnfowDFzcxljvwjeZc(5.0);dbtyvucioxybtnrmJVHBGNFTREIOW();cnjkwnfowDFzcxljvwjeZc(2.0);}void drop_left() {arm_set_height(ARM_FRONT_PLATE);cnjkwnfowDFzcxljvwjeZc(1.0);arm_set_orientation(ARM_LEFT);cnjkwnfowDFzcxljvwjeZc(5.0);bngjfFKFKOQPQPSLCavosoaowej();cnjkwnfowDFzcxljvwjeZc(2.0);}void drop_right() {arm_set_height(ARM_FRONT_PLATE);cnjkwnfowDFzcxljvwjeZc(1.0);arm_set_orientation(ARM_RIGHT);cnjkwnfowDFzcxljvwjeZc(5.0);bngjfFKFKOQPQPSLCavosoaowej();cnjkwnfowDFzcxljvwjeZc(2.0);}void reset_arm() {arm_reset();cnjkwnfowDFzcxljvwjeZc(6.0);}void drop_purple_bin(){drop_right();}void drop_yellow_bin(){drop_left();}int grab_ik(int x, int y){float ALKSDJOQIJvbznpoafsnvm = (float) sqrt(pow((float)x-39.5,2)+pow((float)y-40.0,2));float pobajlnrkjenkavbjnkjwoeij = (float) sqrt(pow((float)x-30.63,2)+pow((float)y-22.44,2));float vxnwucnfeyvbrydidmncow = (float) sqrt(pow((float)x-33.00,2)+pow((float)y-57.29,2));int bmgjigjrjgokonkmwnbhve = 5;if (ALKSDJOQIJvbznpoafsnvm<bmgjigjrjgokonkmwnbhve){ogwrijdlnvjkWEPFVZCNM();return 1;} else if (pobajlnrkjenkavbjnkjwoeij<bmgjigjrjgokonkmwnbhve){VBMFGJKQPOZVCNKsdljfkVJIP(); return 1;} else if (vxnwucnfeyvbrydidmncow<bmgjigjrjgokonkmwnbhve){BEOJWFNKJVSBKNwefjovnkjAVLNJ();return 1;}return -1;}
