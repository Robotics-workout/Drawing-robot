#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <WebServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "config.h"
#include "robot_kinematics.h"
#include "pen_control.h"
#include "stepper_control.h"

// Function declarations
void setupWebServer();
void processVectorPaths(JsonArray paths);
void handleDraw();
void handleRoot();
void handleStatus();

#endif // WEB_SERVER_H

