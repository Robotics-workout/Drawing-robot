#ifndef PEN_CONTROL_H
#define PEN_CONTROL_H

#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "config.h"

// Global servo object
extern Servo penServo;

// Queue and mutex for pen control
extern QueueHandle_t penQueue;
extern SemaphoreHandle_t penStateMutex;

// Current pen state (protected by mutex)
extern PenState currentPenState;

// Function declarations
void penUp();
void penDown();
void penControlTask(void *parameter);
void initPen();

#endif // PEN_CONTROL_H


