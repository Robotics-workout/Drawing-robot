#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define MOTOR_STEPS 200              // Steps per revolution (NEMA 17 = 200 steps/rev, 1.8°/step)
#define MICROSTEPS 1                 // Microstepping level (1 = full step, check your driver settings)
#define PULLEY_TEETH 20              // GT2-20T pulley: 20 teeth
#define GT2_PITCH 2.0                // GT2 belt pitch: 2mm per tooth
#define PULLEY_CIRCUMFERENCE (PULLEY_TEETH * GT2_PITCH)  // 20 × 2mm = 40mm
#define PULLEY_DIAMETER (PULLEY_CIRCUMFERENCE / PI)      // 40mm / π ≈ 12.732mm
#define BASE 700.0              // Distance between motors in mm
#define L_ARM 124.959
#define DRAWING_AREA_HEIGHT 1000.0

#define STEPS_PER_REV (MOTOR_STEPS * MICROSTEPS)
// MM_PER_STEP = circumference / steps per rev = 40mm / 200 = 0.2mm per step
#define MM_PER_STEP (PULLEY_CIRCUMFERENCE / STEPS_PER_REV)

#define SERVO_PIN 19      // Servo for pen up/down
#define PEN_UP_ANGLE 90   // Adjust for your setup
#define PEN_DOWN_ANGLE 0  // Adjust for your setup

#define DRAWING_AREA_X_BIAS 50
#define DRAWING_AREA_Y_BIAS 50

#define LEFT_MOTOR_DIRECTION 1
#define RIGHT_MOTOR_DIRECTION 1

// Stepper motor driver pins (step, direction)
#define RIGHT_STEP_PIN 17
#define RIGHT_DIR_PIN 16
#define LEFT_STEP_PIN 18
#define LEFT_DIR_PIN 5   

// Initialize stepper motors (step, direction pins)
AccelStepper stepper1(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

Servo penServo;  // Create a Servo object

enum PenState {
  PEN_UP,
  PEN_DOWN
};

// Define lengths of belts (in mm)
// TODO: Add a callibration sequence to fix these values
float L1 = 500.0; // Length from motor to left belt gripper
float L2 = 500.0; // Length from motor to right belt gripper
const float drawing_area_x_limits[2] = {DRAWING_AREA_X_BIAS, BASE - DRAWING_AREA_X_BIAS};
const float drawing_area_y_limits[2] = {DRAWING_AREA_Y_BIAS, DRAWING_AREA_HEIGHT - DRAWING_AREA_Y_BIAS};
float Z1_i = L1 + L_ARM; // Initial length from left motor to the pen
float Z2_i = L2 + L_ARM; // Initial length from right motor to the pen

// Absolute step positions (initialized to 0, representing initial belt lengths)
long absoluteSteps1 = 0;
long absoluteSteps2 = 0;

void setup() 
{
  Serial.begin(115200);

  // Setup stepper motors
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  // Attach servo to the defined pin
  penServo.attach(SERVO_PIN);

  penUp(); // Start with pen up

  // TODO: Add calibration sequence to fix L1 and L2 values
}

void loop() 
{
  delay(2000);
  moveTo(50, 50, PenState::PEN_DOWN); // Move to (50, 50) with pen down
  delay(2000);

  moveTo(200, 50, PenState::PEN_DOWN); // Move to (50, 50) with pen down
  delay(2000);

  moveTo(200, 200, PenState::PEN_DOWN); // Move to (50, 50) with pen down
  delay(2000);

  moveTo(50, 200, PenState::PEN_DOWN); // Move to (50, 50) with pen down
  delay(2000);
}

// Move pen to (x, y) using inverse kinematics
// target x, target y and flag to indicate if its drawing or in transit
void moveTo(float x, float y, PenState pen_state) 
{
  float Z1, Z2;
  if (inverseKinematics(x, y, Z1, Z2)) 
  {
    // Check if the pen is down
    if (pen_state == PEN_DOWN) 
    {
      penDown(); // Move pen down
    } 
    else 
    {
      penUp(); // Move pen up
    }
    
    // Calculate relative change in belt length and convert to steps
    long relativeSteps1 = LEFT_MOTOR_DIRECTION * beltToSteps(Z1 - Z1_i);
    long relativeSteps2 = RIGHT_MOTOR_DIRECTION * beltToSteps(Z2 - Z2_i);
    
    // Calculate absolute target positions (current position + relative change)
    long targetSteps1 = absoluteSteps1 + relativeSteps1;
    long targetSteps2 = absoluteSteps2 + relativeSteps2;

    // Calculate absolute distances to scale speeds for synchronized arrival
    long dist1 = abs(relativeSteps1);
    long dist2 = abs(relativeSteps2);
    
    // Base max speed (steps per second)
    float baseMaxSpeed = 1000.0;
    
    // Save original max speeds to restore later
    float originalMaxSpeed1 = stepper1.maxSpeed();
    float originalMaxSpeed2 = stepper2.maxSpeed();
    
    // Scale speeds so both steppers arrive at the same time
    // The stepper with the longer distance gets base speed
    // The other stepper gets proportionally reduced speed
    if (dist1 > dist2 && dist1 > 0) {
      // Stepper1 travels farther - keep it at base speed, slow down stepper2
      stepper1.setMaxSpeed(baseMaxSpeed);
      stepper2.setMaxSpeed(baseMaxSpeed * ((float)dist2 / (float)dist1));
    } else if (dist2 > dist1 && dist2 > 0) {
      // Stepper2 travels farther - keep it at base speed, slow down stepper1
      stepper2.setMaxSpeed(baseMaxSpeed);
      stepper1.setMaxSpeed(baseMaxSpeed * ((float)dist1 / (float)dist2));
    } else {
      // Equal distances or zero movement - use same speed
      stepper1.setMaxSpeed(baseMaxSpeed);
      stepper2.setMaxSpeed(baseMaxSpeed);
    }

    // Move stepper motors to absolute positions with acceleration/deceleration
    stepper1.moveTo(targetSteps1);
    stepper2.moveTo(targetSteps2);

    // Run both steppers with acceleration until they reach their targets
    // This provides smooth acceleration at start and deceleration at end
    // Both will arrive simultaneously due to proportional speed scaling
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
      stepper1.run();
      stepper2.run();
    }
    
    // Restore original max speeds
    stepper1.setMaxSpeed(originalMaxSpeed1);
    stepper2.setMaxSpeed(originalMaxSpeed2);
    
    // Update absolute step positions and belt lengths
    absoluteSteps1 = targetSteps1;
    absoluteSteps2 = targetSteps2;
    Z1_i = Z1;
    Z2_i = Z2;
  } 
  else 
  {
    Serial.println("Position out of reach");
  }
}

// Function to compute motor steps from (x, y)
/*
// Maths
𝜃 = steps moved/steps per radian
𝑑𝑍 = 𝑟pulley×𝜃
steps = 𝑑𝑍/mm per step
*/
bool inverseKinematics(float target_x, float target_y, float &Z1, float &Z2) 
{
  if (target_x < drawing_area_x_limits[0] || target_x > drawing_area_x_limits[1] || 
      target_y < drawing_area_y_limits[0] || target_y > drawing_area_y_limits[1]) {
    return false;
  }
  // Compute arm lengths
  Z1 = sqrt(pow(target_x, 2) + pow(target_y, 2));
  Z2 = sqrt(pow(BASE  - target_x, 2) + pow(target_y, 2));

  return true;
}

// Convert belt movement to motor steps
long beltToSteps(float dZ) {
    return long(dZ / MM_PER_STEP);
}

// Move pen up
void penUp() 
{
  penServo.write(PEN_UP_ANGLE);
}

// Move pen down
void penDown() 
{
  penServo.write(PEN_DOWN_ANGLE);
}
