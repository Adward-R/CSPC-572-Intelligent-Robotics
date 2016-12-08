/*
 * File:
 * Date:
 * Description:
 *
 *
 * Author:
 * Modifications:
 */

#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include <webots/emitter.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define NO_STATES 27

// This factor should be changed depending on how many iterations you find necessary for your algorithm to converge
#define QLEARNING_NO_IT 50

// Gamma is the discounted learning factor - please feel free to play around with this value and talk about its impact on the algorithm in the report
#define GAMMA 0.2

#define MAX_PICKS 3
#define STABLE_NO_IT 20

// IMPORTANT: Please first read comments in main to understand what the variables and matrices defined represent

// Read in rewards matrix
void read_matrices(int R[NO_STATES][NO_STATES],int A[NO_STATES][NO_STATES])
{
    int i, j;
    FILE *fp;
    // Read in reward matrix from reward_matrix.txt into R
    fp = fopen("reward_matrix.txt", "r");
    for (i = 0; i < NO_STATES; i++) {
        for (j = 0; j < NO_STATES; j++) {
            fscanf(fp, "%d", &R[i][j]);
        }
    }
    fclose(fp);
    
    // Read in actions matrix from actions_matrix.txt into A
    fp = fopen("actions_matrix.txt", "r");
    for (i = 0; i < NO_STATES; i++) {
        for (j = 0; j < NO_STATES; j++) {
            fscanf(fp, "%d", &A[i][j]);
        }
    }
    fclose(fp);
    
}

// DO NOT EDIT THIS FUNCTION IN ANY WAY!!!
// Function to reset world to its initial state in order to start a new episode
void reset_world(WbFieldRef red_block_trans,WbFieldRef red_block_rot,WbFieldRef green_block_trans,WbFieldRef green_block_rot,WbFieldRef blue_block_trans,WbFieldRef blue_block_rot,WbDeviceTag emitter)
{
    int reset_arm = 10; // Code 10 = reset arm
    
    // Send message to robot controller to reset arm
    wb_emitter_send(emitter, &reset_arm, sizeof(int));
    
    // Reset the three colored blocks
    const double RED_BLOCK_INIT_TRANS[3] = {0.168628,0.0124917,-0.508039};
    const double RED_BLOCK_INIT_ROT[4] = {-0.680604,0.680604,-0.271211,2.61191};
    wb_supervisor_field_set_sf_vec3f (red_block_trans, RED_BLOCK_INIT_TRANS);
    wb_supervisor_field_set_sf_rotation (red_block_rot, RED_BLOCK_INIT_ROT);
    
    const double GREEN_BLOCK_INIT_TRANS[3] = {0.206919,0.0124917,-0.634445};
    const double GREEN_BLOCK_INIT_ROT[4] = {-0.571289,0.571289,-0.589286,2.07658};
    wb_supervisor_field_set_sf_vec3f (green_block_trans, GREEN_BLOCK_INIT_TRANS);
    wb_supervisor_field_set_sf_rotation (green_block_rot, GREEN_BLOCK_INIT_ROT);
    
    const double BLUE_BLOCK_INIT_TRANS[3] = {0.165166,0.0124917,-0.763325};
    const double BLUE_BLOCK_INIT_ROT[4] = {-0.675041,0.675041,-0.297723,2.56286};
    wb_supervisor_field_set_sf_vec3f (blue_block_trans, BLUE_BLOCK_INIT_TRANS);
    wb_supervisor_field_set_sf_rotation (blue_block_rot, BLUE_BLOCK_INIT_ROT);
}

// Q-learning algorithm for updating Q matrix
void Qlearning_updates(int R[NO_STATES][NO_STATES],int Q[NO_STATES][NO_STATES],int A[NO_STATES][NO_STATES],WbDeviceTag emitter)
{
    int i,j,it;
    int current_state,robot_action;
    
    // Get initial coordinates of objects in the world in order to be able to reset them after each episode finishes
    // You should not change this block of code
    WbNodeRef red_block_node = wb_supervisor_node_get_from_def("RedBlock");
    WbFieldRef red_block_trans = wb_supervisor_node_get_field(red_block_node, "translation");
    WbFieldRef red_block_rot = wb_supervisor_node_get_field(red_block_node, "rotation");
    
    WbNodeRef green_block_node = wb_supervisor_node_get_from_def("GreenBlock");
    WbFieldRef green_block_trans = wb_supervisor_node_get_field(green_block_node, "translation");
    WbFieldRef green_block_rot = wb_supervisor_node_get_field(green_block_node, "rotation");
    
    WbNodeRef blue_block_node = wb_supervisor_node_get_from_def("BlueBlock");
    WbFieldRef blue_block_trans = wb_supervisor_node_get_field(blue_block_node, "translation");
    WbFieldRef blue_block_rot = wb_supervisor_node_get_field(blue_block_node, "rotation");
    
    // Initialize matrix Q to 0s
    // You should not change this block of code
    for (i=0; i<NO_STATES; i++)
        for (j=0; j<NO_STATES; j++)
            Q[i][j] = 0;
    
    // Main loop for each episode
    // You should play with changing how long you need to run the algorithm for by changing the value of QLEARNING_NO_IT
    // You should typically stop when you reach convergence, i.e. when the values in your Q matrix are not changing anymore from iteration to iteration
    // Feel free to stop by checking when your algorithm actually converges, by replacing the for loop with a while/do-while loop
    
    int n_stable_it = 0, n_actions = 0, max, k;
    for (it=0; it<QLEARNING_NO_IT; it++)
    {
        printf("%d\n", it);
        // Each episode starts from state 0
        current_state = 0;

        // NEW: 
        n_actions = 0;
        int prevQ[NO_STATES][NO_STATES];
        memcpy(prevQ, Q, sizeof(int)*NO_STATES*NO_STATES);

        do {
            // This do loop should exit whenever the end of an episode is reached, i.e. after the robot performs 3 movements, starting with all three blocks on the starting mat, and ending with all three on one of the yellow or purple mat (this includes all the end-positions and the one goal position; the end positions, including the goal position, are states 19-26, inclusive)
            // This loop also needs to contain the update rule for the values in the Q matrix, as per the tutorial you need to read
            int next_state;
            do {
                next_state = rand() % NO_STATES;
            } while (R[current_state][next_state] == -1);

            // After you select a random action that can be performed from your current state, you can send it to the robot arm to be performed with the function below; for now, the stub code sends the same value (1) over and over again to the arm, but you will need to change this in your code
            robot_action = A[current_state][next_state];
            wb_emitter_send(emitter, (void *) &robot_action, sizeof(int));

            // NEW
            n_actions++;
            max = 0;
            for (k = 0; k < NO_STATES; k++) {
                if (Q[next_state][k] > max) {
                    max = Q[next_state][k];
                }
            }
            Q[current_state][next_state] = R[current_state][next_state] + GAMMA * max;
            current_state = next_state;
            
        } while (wb_robot_step(TIME_STEP*1000) != -1 && n_actions < MAX_PICKS); // Iterate while we have not reached an end state (end states include the goal state); you will need to add the condition to stop when an end-state has been reached by adding an AND to the while condition
        // For now, the arm will continue doing the same movement picking up the red block just once; the next times it tries to pick it up, the red block will not be in the starting position anymore since it's not being reset;
        // When you implement the proper exit condition for the while loop, the robot will pick up all the blocks (sequentially), after which the objects will be reset through the statement below

        // Reset the world before proceeding to next episode
        // You should not change this line of code
        reset_world(red_block_trans,red_block_rot,green_block_trans,green_block_rot,blue_block_trans,blue_block_rot,emitter);

        // Compare Q to prevQ to see if stablized
        int identical = 1;
        for (i = 0; i < NO_STATES; i++) {
            for (j = 0; j < NO_STATES; j++) {
                if (Q[i][j] != prevQ[i][j]) { identical = 0; break; }
            }
            if (j < NO_STATES) { break; }
        }

        if (identical == 1) { 
            n_stable_it ++;
            if (n_stable_it > STABLE_NO_IT) { break; }
        } else { n_stable_it = 0; }
    }
}

// State recovery from Q matrix
void Qlearning_state_recovery(int Q[NO_STATES][NO_STATES],int state_sequence[3])
{
    int current_state, i, j, max, next_state = 0;
    
    // Set current state to initial state
    current_state = 0;
   
    // Use Q matrix to recover the sequence of states with the highest values
    // You will always start from state 0 and perform 3 actions, so the recovered state of sequences will always have 3 states
    // In this function, you only need to populate the array state_sequence; you shouldn't print this array in this function, this functionality is accomplished in main 
    for (i = 0; i < 3; i++) {
        max = -1;
        for (j = 0; j < NO_STATES; j++) {
            if (Q[current_state][j] > max) {
                max = Q[current_state][j];
                next_state = j;
            }
        }
        current_state = next_state;
        state_sequence[i] = next_state;
    } 
}

int main(int argc, char **argv)
{
    // You should not change any code in main
    // You can add print statements as necessary
    
    // Necessary to initialize webots stuff
    wb_robot_init();
    srand(time(NULL));
    
    while (wb_robot_step(TIME_STEP) == -1);
    
    // Set up emitter
    WbDeviceTag emitter = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter,1);
    wb_emitter_set_range(emitter,-1);
    
    int R[NO_STATES][NO_STATES]; // Rewards matrix
    int Q[NO_STATES][NO_STATES]; // Q values matrix
    int A[NO_STATES][NO_STATES]; // Actions matrix
    int state_sequence[3]; // Sequence of states to be performed to reach goal state
    
    read_matrices(R,A);
    
    Qlearning_updates(R,Q,A,emitter);
    
    Qlearning_state_recovery(Q,state_sequence);
    
    int i;
    printf("The recovered state sequence is:\n");
    for (i = 0; i<3; i++)
        printf("%d ", state_sequence[i]);
    printf("\n");
    
    // This is necessary to cleanup webots resources
    wb_robot_cleanup();
    
    return 0;
}
