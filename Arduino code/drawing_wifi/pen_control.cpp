#include "pen_control.h"

// Global servo object
Servo penServo;

// Queue and mutex (defined in main file)
extern QueueHandle_t penQueue;
extern SemaphoreHandle_t penStateMutex;

// Current pen state (protected by mutex)
PenState currentPenState = PEN_UP;

void initPen() {
  penServo.attach(SERVO_PIN);
  // Set pen to up position directly (mutex not initialized yet)
  penServo.write(PEN_UP_ANGLE);
  currentPenState = PEN_UP;
}

void penUp() {
  penServo.write(PEN_UP_ANGLE);
  if (xSemaphoreTake(penStateMutex, portMAX_DELAY)) {
    currentPenState = PEN_UP;
    xSemaphoreGive(penStateMutex);
  }
}

void penDown() {
  penServo.write(PEN_DOWN_ANGLE);
  if (xSemaphoreTake(penStateMutex, portMAX_DELAY)) {
    currentPenState = PEN_DOWN;
    xSemaphoreGive(penStateMutex);
  }
}

// Pen control task running on Core 1
void penControlTask(void *parameter) {
  PenCommand penCmd;
  
  while (true) {
    if (xQueueReceive(penQueue, &penCmd, portMAX_DELAY)) {
      if (penCmd.isValid) {
        if (penCmd.state == PEN_DOWN) {
          penDown();
        } else {
          penUp();
        }
        delay(50); // Small delay for servo to move
      }
    }
  }
}

