#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>
#include <webots/gyro.h>


#define SPEED 4

int main() 
  {
    // Define
    
    WbDeviceTag leftF_motor, leftR_motor, rightF_motor, rightR_motor, ds, ps; // objects device tags
    int left_speed, right_speed; // motors speed values
    int i, j;
    int rotation_counter = 0;
    double motor_position = 40;
    bool findState = false; // object find state value
    bool rotState = false; // robot rotate value
    
    
    wb_robot_init();
    
    const int time_step = wb_robot_get_basic_time_step();       
  
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
    WbNodeRef target_node = wb_supervisor_node_get_from_def("TARGET");
    WbNodeRef floor_node = wb_supervisor_node_get_from_def("FLOOR");
  
    WbFieldRef trans_field_robot = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef rot_field_robot = wb_supervisor_node_get_field(robot_node, "rotation");
    WbFieldRef trans_field_target = wb_supervisor_node_get_field(target_node, "translation");
    WbFieldRef floorSize_field_floor = wb_supervisor_node_get_field(floor_node, "floorSize");
  
    if (robot_node == NULL) 
      {
        fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
        exit(1);
      }
    if (target_node == NULL) 
      {
        fprintf(stderr, "No DEF TARGET node found in the current world file\n");
        exit(1);
      }
    if (floor_node == NULL) 
      {
        fprintf(stderr, "No DEF FLOOR node found in the current world file\n");
        exit(1);
      }
  
  
    // distance sensors define
    ds = wb_robot_get_device("ds");
    wb_distance_sensor_enable(ds, time_step);
    
    ps = wb_robot_get_device("ps");
    wb_position_sensor_enable(ps, time_step);
  
    // motors define
    leftF_motor = wb_robot_get_device("leftF");
    leftR_motor = wb_robot_get_device("leftR");
    rightF_motor = wb_robot_get_device("rightF");
    rightR_motor = wb_robot_get_device("rightR");
  
    // motors position set
   
  
    // set motors starting velocity
    wb_motor_set_velocity(leftF_motor, 0);
    wb_motor_set_velocity(leftR_motor, 0);
    wb_motor_set_velocity(rightF_motor, 0);
    wb_motor_set_velocity(rightR_motor, 0);
  
    while (wb_robot_step(time_step) != -1 && findState == false) 
      {
        // basic movement go forward
        if(rotState == false)
          {
            right_speed = SPEED;
            left_speed = SPEED;
          }
                
        // get robot position values(x,y,z)
        const double *robot_pos_values = wb_supervisor_field_get_sf_vec3f(trans_field_robot);
        
        // get target object position values(x,y,z)
        const double *target_pos_values = wb_supervisor_field_get_sf_vec3f(trans_field_target);
        
        // get floor object size value(x, y)
        const double *floor_size_values = wb_supervisor_field_get_sf_vec2f(floorSize_field_floor);
        
        const double position = wb_position_sensor_get_value(ps);
        printf("%f\n", position);
        
        const double *robot_rot_values = wb_supervisor_field_get_sf_rotation(rot_field_robot);
        printf("Robot Rotation Values : %f, %f, %f, %f \n", robot_rot_values[0], robot_rot_values[1], robot_rot_values[2], robot_rot_values[3]);
         
        // check robot position and run process
        if(robot_pos_values[0]<target_pos_values[0]+0.1 && robot_pos_values[0]>target_pos_values[0]-0.1 && robot_pos_values[2]<target_pos_values[2]+0.1 && robot_pos_values[2]>target_pos_values[2]-0.1)
        {
          left_speed = 0;
          right_speed = 0;
          fprintf(stderr, "buldum\n");
          findState = true;
        }
        
        
               
        if(position == 40)
          {
            // rotState = true;
            motor_position = INFINITY;
            right_speed = -SPEED+3;
            left_speed = SPEED-3;
            rotState = true;
            printf("1.if\n");
          }
        
        if (robot_rot_values[1] > 0.0 && robot_rot_values[3] > 0.0 && rotState == true)
          {
            // right_speed = SPEED;
            // left_speed = SPEED;
            // motor_position =+ 40;
            printf("2.if\n");
            // rotState = false;
            // printf("rotation END!!\n");
            
          }
        else if(rotState == true)
          {
            // right_speed = -SPEED;
            // left_speed = SPEED;
            
            // define the variable from which distance sensor values will be taken
            // double dsVal;
            
            // get distance sensor values
            // dsVal = wb_distance_sensor_get_value(ds);
            
            // check the distance sensor values and start the rotation
            // if(dsVal < 950)
              // rotation_counter = 30;
          }
      
           
        if (findState == true)
          {
            left_speed = 0;
            right_speed = 0;
          }
        
        
        
        // double dsVal;
        
        
        // dsVal = wb_distance_sensor_get_value(ds);
        
        
        // if(dsVal < 950.0)
        // {
          // left_speed = SPEED;
          // right_speed = -SPEED;
          // pause_counter = 100;
          // rotState = true;
        // }
        
        wb_motor_set_position(leftF_motor, motor_position);
        wb_motor_set_position(leftR_motor, motor_position);
        wb_motor_set_position(rightF_motor, motor_position);
        wb_motor_set_position(rightR_motor, motor_position);
            
        // Set the motor speeds according to final state.
        wb_motor_set_velocity(leftF_motor, left_speed);
        wb_motor_set_velocity(leftR_motor, left_speed);
        wb_motor_set_velocity(rightF_motor, right_speed);
        wb_motor_set_velocity(rightR_motor, right_speed);
        
      } // end of while loop 
  
    wb_robot_cleanup();
  
    return 0;
    
  } // end of main function