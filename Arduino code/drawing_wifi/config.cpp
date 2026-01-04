#include "config.h"

// ============ WiFi CONFIGURATION DEFINITIONS ============
// Option 1: Connect to existing WiFi network (Station mode)
// Set these to your WiFi network credentials, or leave empty to use Access Point mode
const char* wifiSSID = "WiFi2367niq";              // Your WiFi network name (leave empty for AP mode)
const char* wifiPassword = "JDKb7NhZU";           // Your WiFi network password

// Option 2: Access Point mode (fallback or if wifiSSID is empty)
const char* apSSID = "DrawingRobot";    // WiFi Access Point name
const char* apPassword = "draw1234";      // WiFi Access Point password (min 8 chars)

const char* otaPassword = "draw1234";   // OTA update password


