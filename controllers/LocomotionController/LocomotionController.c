#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/connector.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define CONTROL_STEP 32
#define N 8

static double t = 0.0;                 /* time elapsed since simulation start [s] */

/* each module is equipped with a single motormotor and 2 connectors */
static WbDeviceTag motor, rear_connector, front_connector;

double amplitude, offset, phase, frequency;

void updateJoint()
{

}

int main() {
  
  /* necessary to initialize Webots */
  wb_robot_init();
  
  /*
   * Find module id from robot name
   * The robot name is used to identify each module
   */
  const char *name = wb_robot_get_name();
  char * file_name = (char *) malloc((strlen(name)+4)*sizeof(char));
  sprintf(file_name, "%s.txt", name);
  
  /* Open file */
  
  /* If open, read parameters */
  
  /* Close file */

  /* find hardware devices */
  motor           = wb_robot_get_device("motor");
  rear_connector  = wb_robot_get_device("rear_connector");
  front_connector = wb_robot_get_device("front_connector");

  while(wb_robot_step(CONTROL_STEP)!=-1)
 {
    updateJoint();

    /* computed elapsed time */
    t += CONTROL_STEP / 1000.0;
  }

  wb_robot_cleanup();
  
  return 0;
}
