#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <ElegantOTA.h>
#include <WebServer.h>
#include "config.h"

// Global server object
extern WebServer server;

// Function declarations
void setupWiFi();
void setupOTA();

#endif // WIFI_MANAGER_H

