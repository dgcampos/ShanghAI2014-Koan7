#include <webots/robot.h>
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
        wb_robot_step(20);
    }

    wb_robot_cleanup();
    return 0;
}
