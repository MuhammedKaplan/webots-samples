#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/utils/system.h>
#include <webots/supervisor.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define SPEED 4
enum BLOB_TYPE { RED, NONE };

int main() {
  WbDeviceTag camera, leftF_motor, leftR_motor, rightF_motor, rightR_motor;
  int width, height;
  int left_speed, right_speed;
  int i, j;
  int red, blue, green;
  int pause_counter = -1;
  bool findState = false;
  bool rotState = true;
  WbDeviceTag ds;
  const char *color_name = "red";
  const char *ansi_color = ANSI_COLOR_RED;
  const char *filename = "red_blob.png";
  enum BLOB_TYPE current_blob;
  time_t t;

  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
    exit(1);
  }
  
  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("cam");
  wb_camera_enable(camera, time_step);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);
  
  ds = wb_robot_get_device("ds");
  wb_distance_sensor_enable(ds, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  leftF_motor = wb_robot_get_device("leftF");
  leftR_motor = wb_robot_get_device("leftR");
  rightF_motor = wb_robot_get_device("rightF");
  rightR_motor = wb_robot_get_device("rightR");

  wb_motor_set_position(leftF_motor, INFINITY);
  wb_motor_set_position(leftR_motor, INFINITY);
  wb_motor_set_position(rightF_motor, INFINITY);
  wb_motor_set_position(rightR_motor, INFINITY);

  wb_motor_set_velocity(leftF_motor, 0.0);
  wb_motor_set_velocity(leftR_motor, 0.0);
  wb_motor_set_velocity(rightF_motor, 0.0);
  wb_motor_set_velocity(rightR_motor, 0.0);
  
  bool rot_check = false;
  
  int rot = 35;
  
  /* Main loop */
  while (wb_robot_step(time_step) != -1) {
    /* Get the new camera values */
    const unsigned char *image = wb_camera_get_image(camera);
    
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    
    if(values[0]<0.14 && values[0]>-0.06 && values[2]<0.08 && values[2]>-0.12){
      left_speed = 0;
      right_speed = 0;
      fprintf(stderr, "buldum\n");
      findState = true;
    }
    
    
    
    /* Decrement the pause_counter */
      
      if (pause_counter > 0)
      {
      pause_counter--;
      printf("%d \n",pause_counter);
      }
      
      if (pause_counter == 0) 
      {
      rotState = false;
      }
      
      if(pause_counter == rot - 26)
      {
        left_speed = left_speed / 4;
        right_speed = right_speed / 4;
      }
       

    

    
    
    double dsVal;
    
    dsVal = wb_distance_sensor_get_value(ds);
    
    
    if(rotState == true && rot_check == false)
    {
      left_speed = SPEED;
      right_speed = -SPEED;
      pause_counter = rot;
      printf("%d \n",pause_counter);
      //printf("%d", pause_counter*time_step);
      rot_check = true;
      
    }
    if(rotState == false)
    {
      left_speed = 0;
      right_speed = 0;
    }
    
    
    
    /* Set the motor speeds. */
    wb_motor_set_velocity(leftF_motor, left_speed);
    wb_motor_set_velocity(leftR_motor, left_speed);
    wb_motor_set_velocity(rightF_motor, right_speed);
    wb_motor_set_velocity(rightR_motor, right_speed);
    
    wb_motor_set_position(leftF_motor, INFINITY);
  wb_motor_set_position(leftR_motor, INFINITY);
  wb_motor_set_position(rightF_motor, INFINITY);
  wb_motor_set_position(rightR_motor, INFINITY);
  }

  wb_robot_cleanup();

  return 0;
}
