# Drawing Robot WiFi - Code Structure

This directory contains the modularized code for the WiFi-enabled drawing robot.

## File Structure

```
drawing_wifi/
├── drawing_wifi.ino      # Main file - includes all modules and sets up tasks
├── config.h              # Configuration constants, pin definitions, WiFi settings
├── robot_kinematics.h    # Inverse kinematics calculations (header)
├── robot_kinematics.cpp  # Inverse kinematics implementation
├── stepper_control.h     # Stepper motor control (header)
├── stepper_control.cpp   # Stepper motor control implementation
├── pen_control.h         # Pen/servo control (header)
├── pen_control.cpp       # Pen/servo control implementation
├── wifi_manager.h        # WiFi and OTA setup (header)
├── wifi_manager.cpp      # WiFi and OTA implementation
├── web_server.h          # HTTP server and endpoints (header)
└── web_server.cpp        # HTTP server implementation
```

## Module Descriptions

### `config.h`
- Pin definitions (stepper motors, servo)
- Robot parameters (BASE, L_ARM, drawing area dimensions)
- WiFi configuration (SSID, passwords)
- Data structures (PenState enum, StepperMoveCommand, PenCommand)

### `robot_kinematics.h/cpp`
- Inverse kinematics calculations
- Drawing area limits
- Belt length tracking (L1, L2, Z1_i, Z2_i)
- Coordinate validation

### `stepper_control.h/cpp`
- Stepper motor initialization
- Stepper control task (runs on Core 0)
- Continuous motor stepping for precise timing
- Queue-based command processing

### `pen_control.h/cpp`
- Servo initialization
- Pen up/down functions
- Pen control task (runs on Core 1)
- Thread-safe pen state management

### `wifi_manager.h/cpp`
- WiFi Access Point setup
- WiFi Station mode (connect to existing network)
- OTA (Over-The-Air) update configuration
- Connection status reporting

### `web_server.h/cpp`
- HTTP server setup
- REST API endpoints (/draw, /status, /)
- Vector path processing
- Automatic scaling to fit drawing area
- JSON parsing and validation

### `drawing_wifi.ino`
- Main entry point
- Includes all modules
- Creates FreeRTOS tasks
- Sets up inter-core communication (queues, mutexes)
- Minimal code - delegates to modules

## Architecture

### Dual-Core Design
- **Core 0**: Stepper motor control (time-sensitive, continuous loop)
- **Core 1**: WiFi server, pen control, OTA updates

### Inter-Core Communication
- `stepperQueue`: Commands from Core 1 → Core 0 (move positions)
- `penQueue`: Pen commands on Core 1
- `penStateMutex`: Thread-safe pen state access

## Usage

1. Open `drawing_wifi.ino` in Arduino IDE
2. All `.h` and `.cpp` files in the same directory will be automatically compiled
3. Upload to ESP32 as normal

## Customization

### Change Pin Assignments
Edit `config.h`:
```cpp
#define LEFT_STEP_PIN 17
#define SERVO_PIN 19
// etc.
```

### Adjust Robot Parameters
Edit `config.h`:
```cpp
#define BASE 700.0
#define DRAWING_AREA_HEIGHT 1000.0
// etc.
```

### Configure WiFi
Edit `config.h`:
```cpp
const char* wifiSSID = "YourNetwork";
const char* wifiPassword = "YourPassword";
```

## Benefits of Modular Structure

1. **Organization**: Each module has a clear responsibility
2. **Maintainability**: Easy to find and modify specific functionality
3. **Reusability**: Modules can be reused in other projects
4. **Testing**: Individual modules can be tested separately
5. **Readability**: Smaller files are easier to understand
6. **Collaboration**: Multiple developers can work on different modules

## Adding New Features

To add a new feature:
1. Create new `.h` and `.cpp` files (or add to existing module)
2. Include the header in `drawing_wifi.ino`
3. Call initialization functions in `setup()`
4. Add tasks if needed

Example:
```cpp
// new_feature.h
#ifndef NEW_FEATURE_H
#define NEW_FEATURE_H
void initNewFeature();
void newFeatureTask(void *parameter);
#endif

// new_feature.cpp
#include "new_feature.h"
void initNewFeature() { /* ... */ }
void newFeatureTask(void *parameter) { /* ... */ }

// drawing_wifi.ino
#include "new_feature.h"
void setup() {
  initNewFeature();
  xTaskCreatePinnedToCore(newFeatureTask, ...);
}
```


