/*
 * Drawing Robot WiFi Controller
 * 
 * Main file that includes all modules and sets up the dual-core FreeRTOS tasks.
 * 
 * Architecture:
 * - Core 0: Stepper motor control (time-sensitive)
 * - Core 1: WiFi, Pen control, OTA updates
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Include all modules
#include "config.h"
#include "robot_kinematics.h"
#include "stepper_control.h"
#include "pen_control.h"
#include "wifi_manager.h"
#include "web_server.h"

// ============ GLOBAL OBJECTS ============
WebServer server(80);

// Queues for inter-core communication
QueueHandle_t stepperQueue;    // Core 1 -> Core 0: Move commands
QueueHandle_t penQueue;        // Core 1 -> Core 1: Pen commands (internal to Core 1)
SemaphoreHandle_t penStateMutex; // Mutex for coordinating pen state with stepper moves

// ============ TASK FUNCTIONS ============

// WiFi and OTA task running on Core 1
void wifiTask(void *parameter) {
  setupWiFi();
  setupWebServer();
  setupOTA();  // Must be called after setupWebServer() since ElegantOTA uses the server
  
  while (true) {
    server.handleClient();
    ElegantOTA.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ============ SETUP AND MAIN LOOP ============
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Drawing Robot WiFi Controller ===");
  Serial.println("Core 0: Stepper Motor Control (time-sensitive)");
  Serial.println("Core 1: WiFi, Pen Control, OTA Updates");
  
  // Initialize hardware
  initSteppers();
  initPen();
  
  // Create queues
  stepperQueue = xQueueCreate(100, sizeof(StepperMoveCommand));
  penQueue = xQueueCreate(50, sizeof(PenCommand));
  penStateMutex = xSemaphoreCreateMutex();
  
  if (stepperQueue == NULL || penQueue == NULL || penStateMutex == NULL) {
    Serial.println("Error creating queues/mutex!");
    while(1) delay(1000);
  }
  
  // Pin stepper control task to Core 0 (time-sensitive)
  xTaskCreatePinnedToCore(
    stepperControlTask,    // Task function
    "StepperControl",      // Task name
    8192,                  // Stack size (bytes)
    NULL,                  // Parameter
    2,                     // Higher priority for time-sensitive stepper control
    NULL,                  // Task handle
    0                      // Core ID (Core 0)
  );
  
  // Pin pen control task to Core 1
  xTaskCreatePinnedToCore(
    penControlTask,        // Task function
    "PenControl",          // Task name
    4096,                  // Stack size (bytes)
    NULL,                  // Parameter
    1,                     // Priority
    NULL,                  // Task handle
    1                      // Core ID (Core 1)
  );
  
  // Pin WiFi/OTA task to Core 1
  xTaskCreatePinnedToCore(
    wifiTask,              // Task function
    "WiFiTask",            // Task name
    16384,                 // Stack size (bytes) - larger for OTA
    NULL,                  // Parameter
    1,                     // Priority
    NULL,                  // Task handle
    1                      // Core ID (Core 1)
  );
  
  Serial.println("Setup complete. Tasks started on both cores.");
  Serial.println("Core 0: Stepper motor control (continuous)");
  Serial.println("Core 1: Pen control, WiFi server, OTA updates");
}

void loop() {
  // Main loop is empty - everything runs in tasks
  delay(1000);
}
