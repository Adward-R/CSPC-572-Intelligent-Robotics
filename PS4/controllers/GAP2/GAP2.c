/*
 * File:          GA.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
 
#define NUM_TIME_STEPS 1000
#define NUM_GENERATIONS 35
#define POPULATION_SIZE 10
#define N_RANK_SLICE 36
 
#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <limits.h>

typedef struct {
  /*The six floats are, in order, a, b and c from
  the left sensor to the right wheel, then a, b,
  and c from the right sensor to the left wheel*/
  float genes[6];
  float fitness;
} vehicle;

char palette[N_RANK_SLICE];


/*
Computes fitness function. Takes in two 2d float array
in the form float path[t][3] & trace[t][3] to along with
*/
float fitness(float path[NUM_TIME_STEPS][3], 
              float trace[NUM_TIME_STEPS][3]){
    int t;
    float error = 0;
    
    for (t = 0; t < NUM_TIME_STEPS; t++) {
      error += sqrt(pow(trace[t][0] - path[t][0], 2) + pow(trace[t][2] - path[t][2], 2));
    }
    return error;
}

float rand_gen() {
    return ((float) rand()) / ((float) RAND_MAX);
}

/*
 * Takes in a vehicle, v, and returns that same vehicle
 * after being transformed by the mutation process
 */
vehicle mutate(vehicle v){
    int i;
    for (i = 0; i < 2; i++) {
      v.genes[rand()%6] += 2 * rand_gen() - 1;
    }
    //for (i = 0; i < 6; i++) {
        //v.genes[i] /= 10;
        //if (rand_gen() < 0.8) {
            //if (rand_gen() < 0.7) {
                //v.genes[i] += (rand_gen() - 1) * 2;
                //if (v.genes[i] > (i%3)+1 || v.genes[i] < -(i%3)-1) {
                    //v.genes[i] /= 2;
                //}
            //} else {
                //v.genes[i] = v.genes[rand() % 6];
            //}
        //} else {
          //v.genes[i] *= 2;
        //}
    //}
    return v;
}

/*
 * Function to cross vehicle v1 with vehicle v2, and 
 * return the offspring
 */
vehicle cross(vehicle v1, vehicle v2){
    int a, b, i;
    do {
        a = rand() % 6;
        b = rand() %  6;
    } while (a < b);
    if (a == b) return v1;
    for (i = a; i <= b; i++) {
        v1.genes[i] = v2.genes[i];
    }
    return v1;
}

/*
 * Function to return the index of the mating parent
 * based on rank selection
 */
void init_sel_palette() {
    int i, p = 0;
    
    for (i = 2; i < POPULATION_SIZE; i++) {
        memset(palette + p, i, i - 1);
        p += i - 1;
    }
}
int rank_selection() {
    return (int) palette[rand() % N_RANK_SLICE];
}

/*
  qsort() helper function.
*/
int cmp(const void* v1, const void* v2) {
  float arg1 = (*(const vehicle*) v1).fitness;
  float arg2 = (*(const vehicle*) v2).fitness;
  if (arg1 < arg2) { return -1; }
  else if (arg1 == arg2) { return 0; }
  else { return 1; }
}

/*
 * Takes in an array of 10 vehicle structures.
 * Must return array of 10 vehicles, which will compose the next generation
 */
vehicle* create_next_generation(vehicle prev_gen[POPULATION_SIZE]) {
    vehicle *next_generation = (vehicle *) malloc(sizeof(vehicle) * POPULATION_SIZE);
    // sort prev_gen in ascending order w.r.t. vehicles' fitness
    qsort(prev_gen, POPULATION_SIZE, sizeof(vehicle), cmp);
    next_generation[0] = prev_gen[0]; // elitism
    next_generation[1] = prev_gen[2];
    int i;
    for(i = 2; i < POPULATION_SIZE; i++) {
        int p1, p2;
    	do {
            p1 = rank_selection();
            p2 = rank_selection();
    	} while (p1 == p2);
    	vehicle v = cross(prev_gen[p1], prev_gen[p2]);
	next_generation[i] = mutate(v);
    }
    return next_generation;
}


int main() {
  wb_robot_init();
  
  //intialize random library
  srand(time(NULL));
  
  // do this once only
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("robot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");


  //set up emitter 
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter,1);
  wb_emitter_set_range(emitter,-1);

  //create the vehicles of the current generation
  vehicle vehicles[POPULATION_SIZE];
  int v=0;
  for (v=0;v<POPULATION_SIZE;v++)
  {
      //initialize vehicle
      int j;
      for (j=0;j<6;j++) vehicles[v].genes[j]=5*(-0.5+(float)rand()/(float)RAND_MAX);
      
      // Fill in your final values here
      //vehicles[v].genes[0] = -1.472348;
      //vehicles[v].genes[1] = -0.759878;
      //vehicles[v].genes[2] = -1.273254;
      //vehicles[v].genes[3] = 0.419756;
      //vehicles[v].genes[4] = -0.165512;
      //vehicles[v].genes[5] = -1.755247;       
           
      vehicles[v].fitness=0;
  }
  
  init_sel_palette();
  // to init trace coordinates for robot to move along with
  FILE *fp;
  float trace[NUM_TIME_STEPS][3];
  int k;
 
  if ((fp = fopen("coordinates.txt", "r")) == NULL) {
      printf("Cannot retrieve track to along!\n");
  }
    
  for (k = 0; k < NUM_TIME_STEPS; k++) {
      fscanf(fp, "%f %f %f\n", &trace[k][0], &trace[k][1], &trace[k][2]);
  }
  fclose(fp);

  int g;
  for (g=0;g<NUM_GENERATIONS;g++)
  {
  
      printf("Generation %d\n",g);
  
      int v;
      for (v=0;v<POPULATION_SIZE;v++)
      {
          //give the vehicle it's appropriate genetics, at which point it should wait
          wb_emitter_send(emitter, vehicles[v].genes, sizeof(vehicles[v].genes));

          //place vehicle back at the start
          //x=-0.3,z=0.3,y=0
          const double INITIAL[3]={-0.3,0,0.3};
          const double INITIAL_ROT[4] = { 0, 1, 0, 0};
          wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
          wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);

          //get the vehicle to begin
          wb_emitter_send(emitter, "s", sizeof("start"));

          float path[NUM_TIME_STEPS][3];
          int p;
          for (p=0;p<(NUM_TIME_STEPS*3);p++) path[p/3][p%3]=0;

          int t; //the timestep per vehicle
          for (t=0;t<NUM_TIME_STEPS; t++) {
            // this is done repeatedly
            const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
            //printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);

            //move coordinates into path
            int d;
            for (d=0;d<3;d++) path[t][d]=trans[d];

            //move on to next simulation step
            wb_robot_step(30);
          }

          float vehicle_fitness = fitness(path, trace);
          vehicles[v].fitness=vehicle_fitness;
          printf("Fitness:%f Genes: %f %f %f %f %f %f\n",vehicle_fitness,
              vehicles[v].genes[0],vehicles[v].genes[1],vehicles[v].genes[2],
              vehicles[v].genes[3],vehicles[v].genes[4],vehicles[v].genes[5]);
      }

      //By now, we have run all of the vehicles in the current generation.

      vehicle *new_vehicles = create_next_generation(vehicles);   
      memcpy(vehicles,new_vehicles,sizeof(vehicle)*POPULATION_SIZE);
      free(new_vehicles);
 
  }
  
  //Print out the fittest vehicle
  printf("After %d generations, each with %d population, the best vehicle\n",NUM_GENERATIONS,POPULATION_SIZE);

  float highest_fitness=INT_MAX;
  vehicle *best_vehicle;
  for(v=0;v<POPULATION_SIZE;v++)
  {
    if(vehicles[v].fitness<highest_fitness){
       best_vehicle = &(vehicles[v]); 
       highest_fitness = vehicles[v].fitness;  
    }
  }

  printf("has fitness %f and has genes: ",highest_fitness);
  int i;for(i=0;i<6;i++) printf("%f,",(*best_vehicle).genes[i]);
  printf("\n");

  return 0;
}
