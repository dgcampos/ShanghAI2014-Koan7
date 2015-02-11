/*#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>

int main()
{
wb_robot_init();

FILE * file = fopen("dummy.txt", "w");

if (file != NULL)
{
fprintf(file, "Holaaa\n");
fclose(file);
}

while(1) {
wb_robot_step(32);
}

wb_robot_cleanup();
return 0;
}*/

#include <stdio.h>
//#include <string>
//#include <iostream>
#include <math.h>
#include <sys/time.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

//using namespace std;

int main(int argc, char* argv[])
{

  int evalTime;
  struct timeval startTime, endTime;

  //read evaluation time from console input
  //if(argc == 2){
  //evalTime = atoi (argv[1]);
  //} else {
  //set a default value for the evaluation time = 30s
  evalTime = 30;
  //}

  wb_robot_init();

  //create the file for fitness value
  FILE * file = fopen("fitness.txt", "w");

  if(file == NULL){
    return 1;
  }
  double yInit, zInit, y, z;
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("module_4");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  const double *initialPos = wb_supervisor_field_get_sf_vec3f(trans_field);
  yInit = initialPos[1];
  zInit = initialPos[2];


  //run the simulation

  gettimeofday(&startTime, NULL);
  gettimeofday(&endTime, NULL);
  while(endTime.tv_sec - startTime.tv_sec < evalTime){
    wb_robot_step(32);
    gettimeofday(&endTime, NULL);
  }
  //get the distance walked in Z-Y
  const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
  y = trans[1];
  z = trans[2];
  double distance = sqrt((pow((z-zInit), 2) + pow((y-yInit), 2)));

  //write the distance into the txt file
  fprintf(file, "%f", distance);
  fclose(file);

  wb_robot_cleanup();
  return 0;
}
