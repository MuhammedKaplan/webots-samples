#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/utils/system.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define SPEED 4
enum BLOB_TYPE { RED, NONE };

int main() {
  WbDeviceTag camera, leftF_motor, leftR_motor, rightF_motor, rightR_motor;
  int width, height;
  //int pause_counter = 0;
  int left_speed, right_speed;
  int i, j;
  int red, blue, green;
  int pause_counter = 0;
  bool findState = false;
  bool rotState = false;
  //const char *dsName = "disSensor";
  WbDeviceTag ds;
  const char *color_name = "red";
  const char *ansi_color = ANSI_COLOR_RED;
  const char *filename = "red_blob.png";
  enum BLOB_TYPE current_blob;
  time_t t;

  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();

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

  /* Main loop */
  while (wb_robot_step(time_step) != -1) {
    /* Get the new camera values */
    const unsigned char *image = wb_camera_get_image(camera);
    
    
    srand((unsigned) time(&t));
    /* Decrement the pause_counter */
//    if (pause_counter > 0)
//      pause_counter--;
      
      if (pause_counter > 0)
      pause_counter--;
      
      if (pause_counter == 0) 
      {
      rotState = false;
      }
      
    /*
     * Case 1
     * A blob was found recently
     * The robot waits in front of it until pause_counter
     * is decremented enough
     */
    if (findState == true) {
      left_speed = 0;
      right_speed = 0;
    }
    /*
     * Case 2
     * A blob was found quite recently
     * The robot begins to turn but don't analyse the image for a while,
     * otherwise the same blob would be found again
     */
//    else if (pause_counter > 0) {
//      left_speed = SPEED;
//      right_speed = SPEED;
//    }
    /*
     * Case 3
     * The robot turns and analyse the camera image in order
     * to find a new blob
     */
    else if (!image) {  // image may be NULL if Robot.synchronization is FALSE
      left_speed = 0;
      right_speed = 0;
      printf("Image:: NULL!!\n");
    } else {  // pause_counter == 0
      /* Reset the sums */
      red = 0;
      green = 0;
      blue = 0;

      /*
       * Here we analyse the image from the camera. The goal is to detect a
       * blob (a spot of color) of a defined color in the middle of our
       * screen.
       * In order to achieve that we simply parse the image pixels of the
       * center of the image, and sum the color components individually
       */
      for (i = width / 3; i < 2 * width / 3; i++)
      {
        for (j = height / 2; j < 3 * height / 4; j++)
        {
            red += wb_camera_image_get_red(image, width, i, j);
            blue += wb_camera_image_get_blue(image, width, i, j);
            green += wb_camera_image_get_green(image, width, i, j);
        }
      }

      /*
       * If a component is much more represented than the other ones,
       * a blob is detected
       */
      if ((red > 2 * green) && (red > 2 * blue))
        current_blob = RED;
      else
        current_blob = NONE;

      /*
       * Case 3a
       * No blob is detected
       * the robot continues to search
       */
      if (current_blob == NONE) {        
        if(rotState == false)
        {
          left_speed = SPEED-1  ;
          right_speed = SPEED;
        }
      }
      /*
       * Case 3b
       * A blob is detected
       * the robot stops, stores the image, and changes its state
       */
      else {
        left_speed = 0;
        right_speed = 0;
        printf("Looks like I found a %s%s%s blob.\n", ansi_color, color_name, ANSI_COLOR_RESET);
        // compute the file path in the user directory
        char *filepath;

#ifdef _WIN32
        const char *user_directory = wbu_system_short_path(wbu_system_getenv("USERPROFILE"));
        filepath = (char *)malloc(strlen(user_directory) + 16);
        strcpy(filepath, user_directory);
        strcat(filepath, "\\Desktop");
        strcat(filepath, "\\");
#else
        const char *user_directory = wbu_system_getenv("HOME");
        filepath = (char *)malloc(strlen(user_directory) + 16);
        strcpy(filepath, user_directory);
        strcat(filepath, "/");
#endif
        strcat(filepath, filename);
        if(wb_camera_save_image(camera, filepath, 100) != -1)
            printf("Object File Saved...\n");
        else
        {
            printf("Save Failed!!\n");
            exit(-1);
        }
        free(filepath);
        findState = true;
      }
    }
    
    double dsVal;
    
    dsVal = wb_distance_sensor_get_value(ds);
    
    if(dsVal < 950.0 && rotState == false)
    {
      left_speed = SPEED;
      right_speed = -SPEED;
      pause_counter = (rand() % 50)+900 / time_step;
      //printf("%d", pause_counter*time_step);
      rotState = true;
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
