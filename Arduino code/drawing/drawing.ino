#include <AccelStepper.h>
#include <ESP32Servo.h>

#define MOTOR_STEPS 200         // Steps per revolution
#define MICROSTEPS 1            // Microstepping level
#define PULLEY_DIAMETER 12.7    // Pulley diameter in mm
#define BASE 700.0              // Distance between motors in mm
#define L_ARM 124.959
#define DRAWING_AREA_HEIGHT 1000.0

#define STEPS_PER_REV (MOTOR_STEPS * MICROSTEPS)
#define MM_PER_STEP ((PI * PULLEY_DIAMETER) / STEPS_PER_REV)

#define SERVO_PIN 18      // Servo for pen up/down
#define PEN_UP_ANGLE 90   // Adjust for your setup
#define PEN_DOWN_ANGLE 0  // Adjust for your setup

#define DRAWING_AREA_X_BIAS 50
#define DRAWING_AREA_Y_BIAS 50

#define LEFT_MOTOR_DIRECTION 1
#define RIGHT_MOTOR_DIRECTION 1

// Stepper motor driver pins (step, direction)
#define LEFT_STEP_PIN 2
#define LEFT_DIR_PIN 5
#define RIGHT_STEP_PIN 3
#define RIGHT_DIR_PIN 6

// Initialize stepper motors (step, direction pins)
AccelStepper stepper1(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

Servo penServo;  // Create a Servo object

// Define lengths of belts (in mm)
// TODO: Add a callibration sequence to fix these values
float L1 = 500.0; // Length from motor to left belt gripper
float L2 = 500.0; // Length from motor to right belt gripper
const float drawing_area_x_limits[2] = {DRAWING_AREA_X_BIAS, BASE - DRAWING_AREA_X_BIAS};
const float drawing_area_y_limits[2] = {DRAWING_AREA_Y_BIAS, DRAWING_AREA_HEIGHT - DRAWING_AREA_Y_BIAS};
float Z1_i = L1 + L_ARM; // Initial length from left motor to the pen
float Z2_i = L2 + L_ARM; // Initial length from right motor to the pen

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
  moveTo(50, 50);
  delay(2000);
}

// Move pen to (x, y) using inverse kinematics
void moveTo(float x, float y) 
{
  float Z1, Z2;
  if (inverseKinematics(x, y, Z1, Z2)) 
  {
    // Convert belt length change to stepper motor steps
    long steps1 = LEFT_MOTOR_DIRECTION * beltToSteps(Z1 - Z1_i);
    long steps2 = RIGHT_MOTOR_DIRECTION * beltToSteps(Z2 - Z2_i);

    // Move stepper motors
    stepper1.moveTo(steps1);
    stepper2.moveTo(steps2);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) 
    {
      stepper1.run();
      stepper2.run();
    }
    // Set current Z values as the initial values
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
