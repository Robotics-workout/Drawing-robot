#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"
#include "robot_kinematics.h"

// Global stepper objects
extern AccelStepper stepper1;
extern AccelStepper stepper2;

// Queue for stepper commands
extern QueueHandle_t stepperQueue;

// Function declarations
void stepperControlTask(void *parameter);
void initSteppers();

#endif // STEPPER_CONTROL_H

