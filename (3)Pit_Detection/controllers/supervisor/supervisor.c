#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>

int main() {
  WbDeviceTag leftF_motor, leftR_motor, rightF_motor, rightR_motor, ds;
  
  int left_speed, right_speed;
  //int i, j;
  double dsVal;

  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();
  
  ds = wb_robot_get_device("DS");
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

  /* Main loop */
  while (wb_robot_step(time_step) != -1) {
    left_speed = 6;
    right_speed = 6;
    
    dsVal = wb_distance_sensor_get_value(ds);
    
    if(dsVal > 900)
    {
      left_speed = 0;
      right_speed = 0;
    }

    /* Set the motor speeds. */   
    wb_motor_set_velocity(leftF_motor, left_speed);
    wb_motor_set_velocity(leftR_motor, left_speed);
    wb_motor_set_velocity(rightF_motor, right_speed);
    wb_motor_set_velocity(rightR_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
