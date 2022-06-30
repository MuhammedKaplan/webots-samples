#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>
#include <webots/led.h>

#define TIME_STEP 32
#define SPEED 8

int main(int argc, char **argv)
{
  wb_robot_init();

  // Device varaible names
  WbDeviceTag left_front_motor, left_rear_motor, right_front_motor, right_rear_motor, pos_sensor, led;
  // Motors speed values
  int left_speed, right_speed;
  // Movement control switch value
  int movement_count = 0;
  // Target find state value
  bool find_state = false;
  // Range the Robot will go 
  double range = 0;
  // Increase in value in every movement step of the robot
  double increment_range = 43.75;
  // Robot rotation squence value
  int rot_sequence = 0;
  // Sequence check state
  bool seq_check = false;
  // Robot rotate state
  bool rot_state = false;
  
  // Robot led state
  bool red_led = false;
  bool blue_led = false;
  bool green_led = false;
  bool cyan_led = false;

  // Define node references in simulation
  WbNodeRef red_robot_node = wb_supervisor_node_get_from_def("MY_ROBOT_RED");
  WbNodeRef target_node = wb_supervisor_node_get_from_def("TARGET");
  WbNodeRef red_led_node = wb_supervisor_node_get_from_def("RED_LED");
  WbNodeRef blue_led_node = wb_supervisor_node_get_from_def("BLUE_LED");
  WbNodeRef green_led_node = wb_supervisor_node_get_from_def("GREEN_LED");
  WbNodeRef cyan_led_node = wb_supervisor_node_get_from_def("CYAN_LED");  
  
  // Define field references in simulation
  WbFieldRef robot_trans_field = wb_supervisor_node_get_field(red_robot_node, "translation");
  WbFieldRef robot_rot_field = wb_supervisor_node_get_field(red_robot_node, "rotation");
  WbFieldRef target_trans_field = wb_supervisor_node_get_field(target_node, "translation");
  WbFieldRef red_led_on_field = wb_supervisor_node_get_field(red_led_node, "on");
  WbFieldRef blue_led_on_field = wb_supervisor_node_get_field(blue_led_node, "on");
  WbFieldRef green_led_on_field = wb_supervisor_node_get_field(green_led_node, "on");
  WbFieldRef cyan_led_on_field = wb_supervisor_node_get_field(cyan_led_node, "on");
  
  // Check root nodes
  if (red_robot_node == NULL)
  {
    fprintf(stderr, "No DEF MY_ROBOT_RED node found in the current world file\n");
    exit(1);
  }
  if (red_led_node == NULL)
  {
    fprintf(stderr, "No DEF RED_LED node found in the current world file\n");
    exit(1);
  }
  if (blue_led_node == NULL)
  {
    fprintf(stderr, "No DEF BLUE_LED node found in the current world file\n");
    exit(1);
  }
  if (green_led_node == NULL)
  {
    fprintf(stderr, "No DEF GREEN_LED node found in the current world file\n");
    exit(1);
  }
  if (cyan_led_node == NULL)
  {
    fprintf(stderr, "No DEF CYAN_LED node found in the current world file\n");
    exit(1);
  }
  if (target_node == NULL)
  {
    fprintf(stderr, "No DEF TARGET node found in the current world file\n");
    exit(1);
  }
  
  // Get led 
  led = wb_robot_get_device("led");
  
  // Get position sensor
  pos_sensor = wb_robot_get_device("ps");
  wb_position_sensor_enable(pos_sensor, TIME_STEP); // Enable sensor
  
  // Get motors
  left_front_motor = wb_robot_get_device("leftF");
  left_rear_motor = wb_robot_get_device("leftR");
  right_front_motor = wb_robot_get_device("rightF");
  right_rear_motor = wb_robot_get_device("rightR");
    
  while (wb_robot_step(TIME_STEP) != -1 && find_state == false)
  {
    // Get robot position values(x,y,z)
    const double *robot_pos_values = wb_supervisor_field_get_sf_vec3f(robot_trans_field);
    
    // Get target object position values(x,y,z)
    const double *target_pos_values = wb_supervisor_field_get_sf_vec3f(target_trans_field);
    
    // Get robots led state
    red_led = wb_supervisor_field_get_sf_bool(red_led_on_field);
    blue_led = wb_supervisor_field_get_sf_bool(blue_led_on_field);
    green_led = wb_supervisor_field_get_sf_bool(green_led_on_field);
    cyan_led = wb_supervisor_field_get_sf_bool(cyan_led_on_field);
    
    // Get robot rotation values(x,y,z,angle)
    const double *robot_rot_values = wb_supervisor_field_get_sf_rotation(robot_rot_field);
    
    // Get position sensor values
    double ps_val = wb_position_sensor_get_value(pos_sensor);  
    
    // Sequence check statement
    if (seq_check == false)
    {
      // Robot movement loop on duty field
      switch (movement_count)
      {
        /*
            Movement step 0, 1 and 2 initial steps.
            After initial steps go to movement loop.
        */
        case 0:// Go without translation and turn
          range = ps_val + increment_range;
          rot_sequence = 0;
          seq_check = true;
          printf("movement 0\n");
          break;
        
        case 1:// Go without translation and turn
          range = ps_val + increment_range;
          rot_sequence = 1;
          seq_check = true;
          printf("movement 1\n");
          break;
        
        case 2:// Go without translation and turn
          range = ps_val + increment_range;
          rot_sequence = 2;
          seq_check = true;
          printf("movement 2\n");
          break;
        
        case 3:// Go with translation and turn
          increment_range -= 6.25;
          range = ps_val + increment_range;
          rot_sequence = 3;
          seq_check = true;
          printf("movement 3\n");
          break;
        
        case 4:// Go without translation and turn
          range = ps_val + increment_range;
          rot_sequence = 4;
          seq_check = true;
          printf("movement 4\n");
          break;
          
         case 5:// Go with range translation and turn. And turn movement(2)
          increment_range -= 6.25;
          range = ps_val + increment_range;
          rot_sequence = 1;
          seq_check = true;
          printf("movement 5\n");
          break;
      }
    }
    
    // Start turn if robot current position sensor values is greater then range value minus 0.1
    if (ps_val > range-0.1 )
      rot_state = true;
      
    // Set motor speed if robot setting turn state 
    if (rot_state == true)
    {
       left_speed = SPEED/8;
       right_speed = -SPEED/8;
       range = INFINITY;
       printf("turning\n");
    }
    // Else go forward
    if (rot_state == false)
    {
      // Slow down if robot close to turning point
      if(ps_val >= range-3)
      {
        left_speed = SPEED/4;
        right_speed = SPEED/4;
        printf("forward(slow)\n");
      }
      // Else go normal speed
      else
      {
        left_speed = SPEED;
        right_speed = SPEED;
        printf("forward\n");
      }
       
    }
    
    // Angles the robot will turn 
    switch (rot_sequence)
    {
      case 0:
        if(robot_rot_values[1] > 0 
        && robot_rot_values[3] <= 0.02 )
        {
          rot_state = false;
          seq_check = false;
          movement_count = 1;
          printf("rotation 0\n");
        }
        break;
      
      case 1: 
        if(robot_rot_values[1] > 0 
        && robot_rot_values[3] <= -1.55 )
        {
          rot_state = false;
          seq_check = false;
          movement_count = 2;
          printf("rotation 1\n");
        }
        break;
      
      case 2:
        if(robot_rot_values[1] > 0 
        && robot_rot_values[3] <= -3.115 )
        {
          rot_state = false;
          seq_check = false;
          movement_count = 3;
          printf("rotation 2\n");
        }
        break;
      
      case 3:
        if(robot_rot_values[1] > 0 
        && robot_rot_values[3] <= 1.59
        && robot_rot_values[3] > 0 )
        {
          rot_state = false;
          seq_check = false;
          movement_count = 4;
          printf("rotation 3\n");
        }
        break;
      
        // This case same to case 0. But this case used by loop movement
      case 4:
        if(robot_rot_values[1] > 0 
        && robot_rot_values[3] <= 0.02 )
        {
          rot_state = false;
          seq_check = false;
          movement_count = 5;
          printf("rotation 4\n");
        }
        break;
    }
    
    // Check the find target object
    if(robot_pos_values[0] > target_pos_values[0]-0.2 
    && robot_pos_values[0] < target_pos_values[0]+0.2 
    && robot_pos_values[2] > target_pos_values[2]-0.2 
    && robot_pos_values[2] < target_pos_values[2]+0.2)
    {
      fprintf(stderr, "TARGET FOUND!!\n");
      wb_led_set(led, 1);
    }
    
    // Stop robot and exit life loop if one of the robots found the target
    if(red_led == true
    || blue_led == true
    || green_led == true
    || cyan_led == true )
    {
      left_speed = 0;
      right_speed = 0;
      find_state = true;
      printf("All robot are stop!!\n");
    }
    
    // Set motor position every step
    wb_motor_set_position(left_front_motor, range);
    wb_motor_set_position(left_rear_motor, range);
    wb_motor_set_position(right_front_motor, range);
    wb_motor_set_position(right_rear_motor, range);
      
    // Set motor speed every step
    wb_motor_set_velocity(left_front_motor, left_speed);
    wb_motor_set_velocity(left_rear_motor, left_speed);
    wb_motor_set_velocity(right_front_motor, right_speed);
    wb_motor_set_velocity(right_rear_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
