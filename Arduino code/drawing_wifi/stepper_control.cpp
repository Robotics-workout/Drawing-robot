#include "stepper_control.h"

// Global stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

// Queue for stepper commands (defined in main file)
extern QueueHandle_t stepperQueue;

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
    if (xQueueReceive(stepperQueue, &moveCmd, 0)) {  // Non-blocking check
      if (moveCmd.isValid) {
        float Z1, Z2;
        if (inverseKinematics(moveCmd.x, moveCmd.y, Z1, Z2)) {
          // Convert belt length change to stepper motor steps
          long steps1 = LEFT_MOTOR_DIRECTION * beltToSteps(Z1 - Z1_i);
          long steps2 = RIGHT_MOTOR_DIRECTION * beltToSteps(Z2 - Z2_i);
          
          // Set target positions
          stepper1.moveTo(steps1);
          stepper2.moveTo(steps2);
          
          // Update current Z values
          Z1_i = Z1;
          Z2_i = Z2;
        }
      }
    }
    
    // Continuously run steppers (time-critical)
    stepper1.run();
    stepper2.run();
  }
}

