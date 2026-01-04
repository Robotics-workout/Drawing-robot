#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// ============ PIN DEFINITIONS ============
#define SERVO_PIN 19      // Servo for pen up/down
#define PEN_UP_ANGLE 90   // Adjust for your setup
#define PEN_DOWN_ANGLE 0  // Adjust for your setup

// Stepper motor driver pins (step, direction)
#define RIGHT_STEP_PIN 17
#define RIGHT_DIR_PIN 16
#define LEFT_STEP_PIN 18
#define LEFT_DIR_PIN 5

// ============ ROBOT PARAMETERS ============
#define MOTOR_STEPS 200         // Steps per revolution
#define MICROSTEPS 1            // Microstepping level
#define PULLEY_DIAMETER 12.7    // Pulley diameter in mm
#define BASE 700.0              // Distance between motors in mm
#define L_ARM 124.959
#define DRAWING_AREA_HEIGHT 1000.0
#define DRAWING_AREA_X_BIAS 50
#define DRAWING_AREA_Y_BIAS 50
#define LEFT_MOTOR_DIRECTION 1
#define RIGHT_MOTOR_DIRECTION 1

#define STEPS_PER_REV (MOTOR_STEPS * MICROSTEPS)
#define MM_PER_STEP ((PI * PULLEY_DIAMETER) / STEPS_PER_REV)

// ============ WiFi CONFIGURATION ============
// Option 1: Connect to existing WiFi network (Station mode)
// Set these to your WiFi network credentials, or leave empty to use Access Point mode
extern const char* wifiSSID;              // Your WiFi network name (leave empty for AP mode)
extern const char* wifiPassword;           // Your WiFi network password

// Option 2: Access Point mode (fallback or if wifiSSID is empty)
extern const char* apSSID;    // WiFi Access Point name
extern const char* apPassword;      // WiFi Access Point password (min 8 chars)

extern const char* otaPassword;   // OTA update password

// ============ DATA STRUCTURES ============
enum PenState {
  PEN_UP,
  PEN_DOWN
};

struct StepperMoveCommand {
  float x;
  float y;
  bool isValid;
};

struct PenCommand {
  PenState state;
  bool isValid;
};

#endif // CONFIG_H

