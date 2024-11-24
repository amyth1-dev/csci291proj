#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define MIN_STEPS_BEFORE_DETECTING_START 30

// Starting position tolerance
#define X_TOLERANCE 0.05
#define Y_TOLERANCE 0.05
#define Z_TOLERANCE 0.05

// Starting position coordinates
#define START_X -1.38
#define START_Y 0.35
#define START_Z 0.0

// Function to initialize the light sensor
WbDeviceTag initialize_light_sensor() {
  WbDeviceTag light_sensor = wb_robot_get_device("ls0");
  wb_light_sensor_enable(light_sensor, TIME_STEP);
  return light_sensor;
}

// Function to check if the robot is at the starting position
int is_back_at_start(const double *gps_values) {
  return (fabs(gps_values[0] - START_X) <= X_TOLERANCE &&
          fabs(gps_values[1] - START_Y) <= Y_TOLERANCE &&
          fabs(gps_values[2] - START_Z) <= Z_TOLERANCE);
}

int main(int argc, char **argv) {
  // Initialize Webots API
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
  for (int i = 0; i < 8; ++i) {
    sprintf(prox_sensor_name, "ps%d", i);
    prox_sensors[i] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
  }

  // Initialize light sensor
  WbDeviceTag light_sensor = initialize_light_sensor();

  // Initialize GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // Variables to store the brightest light value and its position
  double max_light_value = -1.0;
  double brightest_coords[3] = {0.0, 0.0, 0.0};
  int step_count = 0;

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    step_count++;

    // Read light sensor value
    double light_value = wb_light_sensor_get_value(light_sensor);

    // Read GPS values
    const double *gps_values = wb_gps_get_values(gps);

    // Track the brightest light value and its position
    if (light_value > max_light_value) {
      max_light_value = light_value;
      brightest_coords[0] = gps_values[0];
      brightest_coords[1] = gps_values[1];
      brightest_coords[2] = gps_values[2];
    }

    // Check if the robot is back at the starting position
    if (step_count > MIN_STEPS_BEFORE_DETECTING_START && is_back_at_start(gps_values)) {
      // Stop the robot
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);

      // Display the message and brightest light information
      printf("Back at starting point detected!\n");
      printf("Brightest Light Value: %f at Coordinates (%f, %f, %f)\n",
             max_light_value, brightest_coords[0], brightest_coords[1], brightest_coords[2]);

      // Begin navigation to the brightest point
      printf("Navigating to the brightest point...\n");

      while (wb_robot_step(TIME_STEP) != -1) {
        // Get updated GPS values
        gps_values = wb_gps_get_values(gps);

        // If near the brightest point, stop
        if (fabs(gps_values[0] - brightest_coords[0]) <= X_TOLERANCE &&
            fabs(gps_values[1] - brightest_coords[1]) <= Y_TOLERANCE &&
            fabs(gps_values[2] - brightest_coords[2]) <= Z_TOLERANCE) {
          printf("Reached the brightest point!\n");
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          break;
        }

        // Navigate through the maze using your original maze-solving logic
        bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
        bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;

        double left_speed = 0.0;
        double right_speed = 0.0;

        if (front_wall) {
          // Turn right
          left_speed = MAX_SPEED;
          right_speed = -MAX_SPEED;
        } else if (left_wall) {
          // Go straight
          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED;
        } else {
          // Turn left slightly
          left_speed = MAX_SPEED / 8;
          right_speed = MAX_SPEED;
        }

        // Set motor speeds
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
      }
      break;  // Exit the main loop after reaching the brightest point
    }

    // Maze-solving logic (original)
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;

    double left_speed = 0.0;
    double right_speed = 0.0;

    if (front_wall) {
      // Turn right
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else if (left_wall) {
      // Go straight
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    } else {
      // Turn left slightly
      left_speed = MAX_SPEED / 8;
      right_speed = MAX_SPEED;
    }

    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  // Cleanup Webots API
  wb_robot_cleanup();
  return 0;
}
    