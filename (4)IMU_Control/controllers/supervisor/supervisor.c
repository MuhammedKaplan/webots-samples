#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <webots/inertial_unit.h>

int main() {
  WbDeviceTag leftF_motor, leftR_motor, rightF_motor, rightR_motor, ds, compass, iu;
  
  int left_speed, right_speed;

  wb_robot_init();
  
  const int time_step = wb_robot_get_basic_time_step();
  
  // Define and enable distance sensor
  ds = wb_robot_get_device("DS");
  wb_distance_sensor_enable(ds, time_step);
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, time_step);
  
  iu = wb_robot_get_device("iu");
  wb_inertial_unit_enable(iu, time_step);
  
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
    
    //double dsVal = wb_distance_sensor_get_value(ds);
    
    const double *north = wb_compass_get_values(compass);
    double angle = atan2(north[0], north[2]);
    
    const double *imu = wb_inertial_unit_get_roll_pitch_yaw(iu);
    
    if(imu[0]<-3 || imu[0]>3)
    {
      left_speed = -6;
      right_speed = -6;
    }
    
    
    //printf("%f, %f, %f, %f \n", north[0], north[1], north[2], angle);
    printf("%f, %f, %f\n", imu[0], imu[1], imu[2]);
    
    /* Set the motor speeds. */   
    wb_motor_set_velocity(leftF_motor, left_speed);
    wb_motor_set_velocity(leftR_motor, left_speed);
    wb_motor_set_velocity(rightF_motor, right_speed);
    wb_motor_set_velocity(rightR_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
