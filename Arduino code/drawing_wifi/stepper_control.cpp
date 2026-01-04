#include "stepper_control.h"

// Global stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

MultiStepper stepperControl;

long positionSteps[2];

// Queue for stepper commands (defined in main file)
extern QueueHandle_t stepperQueue;

void initSteppers() {
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  stepperControl.addStepper(stepper1);
  stepperControl.addStepper(stepper2);
}

// Stepper motor control task running on Core 0
// This task runs continuously to ensure precise timing for stepper motors
void stepperControlTask(void *parameter) {
  StepperMoveCommand moveCmd;
  
  // Check for new move commands
  if (xQueueReceive(stepperQueue, &moveCmd, 0)) {  // Non-blocking check
    if (moveCmd.isValid) {
      float Z1, Z2;
      if (inverseKinematics(moveCmd.x, moveCmd.y, Z1, Z2)) {
        // Convert belt length change to stepper motor steps
        long steps1 = LEFT_MOTOR_DIRECTION * beltToSteps(Z1 - Z1_i);
        long steps2 = RIGHT_MOTOR_DIRECTION * beltToSteps(Z2 - Z2_i);
        
        // Move stepper motors
        positionSteps[0] = steps1;
        positionSteps[1] = steps2;

        stepperControl.moveTo(positionSteps);
        stepperControl.runSpeedToPosition();
        
        // Update current Z values
        Z1_i = Z1;
        Z2_i = Z2;

        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
      }
    }
  }
}

