#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
  
#define TIME_STEP 64

// Define sensor indices
#define FRONT_SENSOR 0
#define RIGHT_SENSOR 1
#define LEFT_SENSOR 2

// Declare sensor and motor tags
WbDeviceTag ds[3];       // Array for distance sensors (front, right, left)
WbDeviceTag light_sensor; // Light sensor
WbDeviceTag left_motor, right_motor; // Motors for wheels

// Function to initialize all sensors and motors
void initialize_robot() {
  // Initialize distance sensors
  ds[FRONT_SENSOR] = wb_robot_get_device("ps0");  // Front distance sensor
  ds[RIGHT_SENSOR] = wb_robot_get_device("ps1");  // Right distance sensor
  ds[LEFT_SENSOR] = wb_robot_get_device("ps2");   // Left distance sensor
  wb_distance_sensor_enable(ds[FRONT_SENSOR], TIME_STEP);
  wb_distance_sensor_enable(ds[RIGHT_SENSOR], TIME_STEP);
  wb_distance_sensor_enable(ds[LEFT_SENSOR], TIME_STEP);

  // Initialize light sensor
  light_sensor = wb_robot_get_device("ls0");
  wb_light_sensor_enable(light_sensor, TIME_STEP);

  // Initialize motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);  // Allow continuous rotation
  wb_motor_set_position(right_motor, INFINITY); // Allow continuous rotation
}

// Function to move the robot forward
void move_forward() {
  wb_motor_set_velocity(left_motor, 3.0);
  wb_motor_set_velocity(right_motor, 3.0);
}

// Function to turn the robot to the right
void turn_right() {
  wb_motor_set_velocity(left_motor, 2.5);
  wb_motor_set_velocity(right_motor, -2.5);
}

// Function to turn the robot to the left
void turn_left() {
  wb_motor_set_velocity(left_motor, -2.5);
  wb_motor_set_velocity(right_motor, 2.5);
}

// Function to stop the robot
void stop_robot() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

int main() {
  wb_robot_init();  // Initialize the Webots API
  initialize_robot();  // Call function to initialize sensors and motors
  
  // Main loop for the robot
  while (wb_robot_step(TIME_STEP) != -1) {
    // The loop runs but doesn't perform any actions (logic can be added here later)
  }

  wb_robot_cleanup();  // Cleanup Webots resources before exiting
  return 0;
}
