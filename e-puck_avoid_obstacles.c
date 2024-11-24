#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define LIGHT_TOLERANCE 20.0  // Adjusted for sensor noise
#define SENSOR_TOLERANCE 300.0 // Adjusted for proximity sensors
#define MIN_STEPS_BEFORE_DETECTING_START 30 // Minimum steps to ensure full loop completion

// Function to initialize the light sensor
WbDeviceTag initialize_light_sensor() {
  WbDeviceTag light_sensor = wb_robot_get_device("ls0");
  wb_light_sensor_enable(light_sensor, TIME_STEP);
  return light_sensor;
}

int main(int argc, char **argv) {
  // Initialize the Webots API
  wb_robot_init();

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize proximity sensors
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

  // Initialize the light sensor
  WbDeviceTag light_sensor = initialize_light_sensor();

  // Store initial sensor readings
  double initial_light_value = wb_light_sensor_get_value(light_sensor);
  double initial_prox_values[8];
  for (int i = 0; i < 8; i++) {
    initial_prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
  }

  // Flags and variables
  int step_count = 0;           // Step counter to track movement
  int back_at_start = 0;        // Flag to detect return to start
  int loop_completed = 0;       // Flag to ensure the robot completes at least one full loop
  int navigating_to_light = 0;  // Flag for light navigation phase
  double brightest_light = -1.0; // Track maximum light value
  double brightest_prox_values[8]; // Track sensor values at brightest position

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    step_count++;

    // Read sensors
    double current_light_value = wb_light_sensor_get_value(light_sensor);
    double current_prox_values[8];
    for (int i = 0; i < 8; i++) {
      current_prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
   }

    // Detect if back at the start
    if (step_count > MIN_STEPS_BEFORE_DETECTING_START && !loop_completed) {
      int match = 1;
      for (int i = 0; i < 8; i++) {
        if (fabs(initial_prox_values[i] - current_prox_values[i]) > SENSOR_TOLERANCE) {
         match = 0;
        break;
       }
      }
     if (fabs(initial_light_value - current_light_value) > LIGHT_TOLERANCE) {
      match = 0;
     }

      if (match) {
        printf("Back at start detected after %d steps.\n", step_count);
        back_at_start = 1;
        loop_completed = 1;
     }
   }

    // Track brightest light source
    if (current_light_value > brightest_light) {
      brightest_light = current_light_value;
      for (int i = 0; i < 8; i++) {
        brightest_prox_values[i] = current_prox_values[i];
      }
    }

    // Maze-solving logic
    double left_speed = 0.0;
    double right_speed = 0.0;
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;

    if (!navigating_to_light) {
      // Normal maze-solving
      if (front_wall) {
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
      } else if (left_wall) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        left_speed = MAX_SPEED / 8;
        right_speed = MAX_SPEED;
      }
    } else {
      // Navigate toward brightest light
      if (front_wall) {
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
      } else if (current_light_value < brightest_light) {
        left_speed = MAX_SPEED / 2;
        right_speed = MAX_SPEED / 2;
      } else {
        printf("Reached the brightest point!\n");
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        break;
      }
    }

    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    // Check if navigating to the brightest point
    if (back_at_start && !navigating_to_light) {
      printf("Navigating to the brightest point...\n");
      navigating_to_light = 1;
    }
  }

  // Stop the robot
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Cleanup
  wb_robot_cleanup();

  return 0;
}
