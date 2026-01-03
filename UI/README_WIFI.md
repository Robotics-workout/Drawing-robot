# WiFi Drawing Robot Controller

This directory contains scripts for controlling the drawing robot via WiFi.

## Files

- `drawing_wifi.ino` - ESP32 Arduino code with WiFi support (dual-core)
- `image_to_vectors.py` - Python client to convert images to vectors and send to robot
- `send_waypoints.py` - Python client to send waypoints directly without image processing
- `drawing_simulator.py` - **NEW!** Visual simulation UI to see robot drawing in real-time
- `examples.py` - Interactive tutorial script with usage examples
- `QUICK_START.md` - Quick reference guide for getting started

## Setup

### Arduino Code

1. Install required libraries in Arduino IDE:
   - `AccelStepper` by Mike McCauley
   - `ESP32Servo` by Kevin Harrington
   - `ArduinoJson` by Benoit Blanchon (v6 or v7)
   - `WebServer` (included with ESP32 board package)
   - `ElegantOTA` by Ayush Sharma (install from Library Manager)

2. Configure WiFi in `drawing_wifi.ino`:
   - **Option A - Connect to existing WiFi**: Set `wifiSSID` and `wifiPassword` to your network credentials
   - **Option B - Access Point mode**: Leave `wifiSSID` empty to create a hotspot
   - Default AP settings: SSID `DrawingRobot`, Password `draw1234`

3. Upload `drawing_wifi.ino` to your ESP32

4. The robot will:
   - If `wifiSSID` is set: Connect to your WiFi network (check Serial Monitor for IP address)
   - If `wifiSSID` is empty: Create a WiFi Access Point at `192.168.4.1`

### Python Client

1. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Connect your computer to the same WiFi network:
   - If robot is in AP mode: Connect to `DrawingRobot` network
   - If robot is connected to existing WiFi: Connect to the same network

3. Find the robot's IP address:
   - Check Serial Monitor output when robot starts
   - Or check your router's connected devices list
   - Default AP IP: `192.168.4.1`

4. Run the image converter:
   ```bash
   python image_to_vectors.py path/to/your/image.png --ip <ROBOT_IP>
   ```
   Or use default IP (192.168.4.1) if in AP mode:
   ```bash
   python image_to_vectors.py path/to/your/image.png
   ```

## Quick Start

**New to the system?** Start here:
1. See `QUICK_START.md` for a 5-minute getting started guide
2. Run `python examples.py` for interactive tutorials
3. Check `SETUP_GUIDE.md` (in project root) for detailed setup instructions

## Usage

### Basic Usage

**Convert an image and send to robot:**
```bash
python image_to_vectors.py image.png
```

**Send waypoints directly (no image processing):**
```bash
# From JSON file
python send_waypoints.py waypoints.json

# From command line
python send_waypoints.py --waypoints "100,100 200,100 200,200 100,200 100,100"
```

**Visualize drawing with simulation UI:**
```bash
# Simulate from JSON file
python drawing_simulator.py --file vectors.json

# Simulate from image
python drawing_simulator.py --image logo.png

# Connect to robot and visualize
python drawing_simulator.py --file vectors.json --robot 192.168.4.1 --send
```

### Advanced Options

**Image conversion:**
```bash
# Use threshold-based edge detection instead of Canny
python image_to_vectors.py image.png --method threshold

# Adjust Canny thresholds
python image_to_vectors.py image.png --low-threshold 30 --high-threshold 100

# Only convert, don't send to robot
python image_to_vectors.py image.png --no-send

# Save vectors to JSON file
python image_to_vectors.py image.png --save-json vectors.json

# Specify robot IP (if different)
python image_to_vectors.py image.png --ip 192.168.1.100
```

**Waypoint sending:**
```bash
# Send waypoints without scaling (use coordinates as-is)
python send_waypoints.py waypoints.json --no-scale

# Multiple paths (pipe-separated)
python send_waypoints.py --waypoints "100,100 200,100 200,200 | 300,300 400,300 400,400"

# Save waypoints before sending
python send_waypoints.py waypoints.json --save-json backup.json
```

### Automatic Scaling

**By default, all paths are automatically scaled to fit within the drawing area:**
- Paths maintain their aspect ratio
- Paths are centered in the drawing area
- A 10mm margin is maintained around edges
- This applies to both image conversion and waypoint sending

**To disable scaling** (use coordinates as-is, may be clipped):
```bash
python send_waypoints.py waypoints.json --no-scale
```

### API Endpoints

The robot exposes the following HTTP endpoints:

- `GET /` - Web interface for manual testing
- `POST /draw` - Send vector paths to draw
- `GET /status` - Get robot status and queue sizes

### Over-The-Air (OTA) Updates

The robot supports OTA firmware updates via WiFi using ElegantOTA:

1. Connect to the robot's WiFi network (`DrawingRobot`)
2. Open a web browser and navigate to: `http://192.168.4.1/update`
3. Enter the OTA password: `draw1234` (default, change in code if needed)
4. Select your compiled `.bin` file and upload
5. The robot will automatically reboot after successful upload

**Alternative method using Arduino IDE:**
1. In Arduino IDE, go to **Tools → Port** and select the network port (e.g., `DrawingRobot at 192.168.4.1`)
2. Upload your sketch normally - it will be uploaded over WiFi

**Note**: The first upload must be done via USB. After that, all subsequent updates can be done over WiFi using the web interface or Arduino IDE.

### JSON Format

Send vector paths in the following JSON format:
```json
{
  "paths": [
    [
      [100.0, 100.0],
      [200.0, 100.0],
      [200.0, 200.0],
      [100.0, 200.0],
      [100.0, 100.0]
    ],
    [
      [300.0, 300.0],
      [400.0, 300.0],
      [350.0, 400.0],
      [300.0, 300.0]
    ]
  ]
}
```

Each path is a list of [x, y] coordinates in millimeters.
Coordinates should be within the drawing area:
- X: 50 to 650 mm (BASE - 2*X_BIAS = 700 - 100)
- Y: 50 to 950 mm (DRAWING_AREA_HEIGHT - 2*Y_BIAS = 1000 - 100)

## Architecture

### Dual-Core Design

- **Core 0**: Stepper motor control (time-sensitive, continuous loop)
  - Dedicated to precise stepper motor timing
  - Runs steppers continuously for smooth movement
  - Receives move commands via queue from Core 1

- **Core 1**: WiFi, Pen Control, and OTA Updates
  - WiFi Access Point and HTTP server
  - Servo control for pen up/down
  - Over-the-air (OTA) firmware updates
  - Processes incoming drawing data and coordinates operations

Communication between cores uses FreeRTOS queues and semaphores for thread-safe coordination.

### Image to Vector Conversion

The Python script performs:
1. Image loading and preprocessing (grayscale, resize)
2. Edge detection (Canny or adaptive threshold)
3. Contour tracing
4. Path simplification (Douglas-Peucker algorithm)
5. Coordinate scaling to drawing area

## Troubleshooting

- **Can't connect to WiFi**: Make sure you're connecting to `DrawingRobot` network
- **Connection timeout**: Check that robot IP is `192.168.4.1` (default AP IP)
- **Out of memory**: Reduce image size or simplify vectors (increase `--epsilon`)
- **Poor edge detection**: Try different threshold values or switch to `threshold` method
- **Drawing too detailed**: Increase `--epsilon` for simpler paths, increase `--min-length` to filter small contours

## Customization

### Change WiFi Configuration

**To connect to existing WiFi network:**
Edit in `drawing_wifi.ino`:
```cpp
const char* wifiSSID = "YourWiFiNetwork";
const char* wifiPassword = "YourWiFiPassword";
```

**To use Access Point mode (create hotspot):**
Leave `wifiSSID` empty:
```cpp
const char* wifiSSID = "";  // Empty = AP mode
const char* apSSID = "DrawingRobot";
const char* apPassword = "draw1234";
```

The robot will automatically fall back to AP mode if it cannot connect to the specified WiFi network.

### Adjust Drawing Area

The drawing area is defined by:
- `BASE`: Distance between motors (700 mm)
- `DRAWING_AREA_HEIGHT`: Height of drawing area (1000 mm)
- `DRAWING_AREA_X_BIAS` and `DRAWING_AREA_Y_BIAS`: Safety margins (50 mm each)

Update these values in the Arduino code and adjust `ImageToVectorConverter` parameters accordingly.

