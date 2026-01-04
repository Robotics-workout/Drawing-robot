#include "stepper_control.h"

// Global stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

// Queue for stepper commands (defined in main file)
extern QueueHandle_t stepperQueue;

// Absolute step positions (initialized to 0, representing initial belt lengths)
long absoluteSteps1 = 0;
long absoluteSteps2 = 0;

void initSteppers() {
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
}

// Stepper motor control task running on Core 0
// This task runs continuously to ensure precise timing for stepper motors
void stepperControlTask(void *parameter) {
  StepperMoveCommand moveCmd;
  
  while (true) {
    // Check for new move commands
    if (xQueueReceive(stepperQueue, &moveCmd, portMAX_DELAY)) {  // Blocking wait for commands
      if (moveCmd.isValid) {
        float Z1, Z2;
        if (inverseKinematics(moveCmd.x, moveCmd.y, Z1, Z2)) {
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
      }
    }
  }
}
