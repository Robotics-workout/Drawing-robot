# Drawing Robot Setup and Usage Guide

This guide will walk you through setting up and using the WiFi-enabled drawing robot.

## Table of Contents
1. [Hardware Setup](#hardware-setup)
2. [Arduino IDE Setup](#arduino-ide-setup)
3. [Uploading Code to ESP32](#uploading-code-to-esp32)
4. [Python Client Setup](#python-client-setup)
5. [Using the System](#using-the-system)
6. [Troubleshooting](#troubleshooting)

---

## Hardware Setup

1. **Connect your ESP32** to your computer via USB
2. **Connect stepper motors** to the defined pins:
   - Left motor: Step pin 17, Direction pin 16
   - Right motor: Step pin 5, Direction pin 18
3. **Connect servo** for pen control to pin 19
4. **Power your motors** with appropriate power supply

---

## Arduino IDE Setup

### Step 1: Install Arduino IDE
Download from: https://www.arduino.cc/en/software

### Step 2: Install ESP32 Board Support
1. Open Arduino IDE
2. Go to **File → Preferences**
3. In "Additional Board Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools → Board → Boards Manager**
5. Search for "ESP32" and install "esp32 by Espressif Systems"

### Step 3: Install Required Libraries
Go to **Sketch → Include Library → Manage Libraries** and install:

1. **AccelStepper** by Mike McCauley
   - Search: "AccelStepper"
   - Install: "AccelStepper by Mike McCauley"

2. **ESP32Servo** by Kevin Harrington
   - Search: "ESP32Servo"
   - Install: "ESP32Servo by Kevin Harrington"

3. **ArduinoJson** by Benoit Blanchon
   - Search: "ArduinoJson"
   - Install: "ArduinoJson by Benoit Blanchon" (version 6.x or 7.x)

4. **ElegantOTA** by Ayush Sharma
   - Search: "ElegantOTA"
   - Install: "ElegantOTA by Ayush Sharma"

### Step 4: Configure Board Settings
1. Go to **Tools → Board → ESP32 Arduino**
2. Select your ESP32 board (e.g., "ESP32 Dev Module")
3. Set **Upload Speed**: 115200 (or higher if supported)
4. Set **CPU Frequency**: 240MHz (recommended)
5. Set **Flash Frequency**: 80MHz
6. Set **Flash Size**: 4MB (or your board's size)
7. Set **Partition Scheme**: Default 4MB with spiffs
8. Select your **Port**: Tools → Port → (your ESP32 port)

---

## Uploading Code to ESP32

### First Time Upload (USB)

1. **Open the sketch**: `Arduino code/drawing/drawing_wifi.ino`

2. **Configure WiFi** (optional):
   - To connect to existing WiFi, edit lines 39-40:
     ```cpp
     const char* wifiSSID = "YourWiFiNetwork";
     const char* wifiPassword = "YourWiFiPassword";
     ```
   - To use Access Point mode, leave `wifiSSID` empty:
     ```cpp
     const char* wifiSSID = "";  // Creates hotspot
     ```

3. **Verify the code**: Click the ✓ button or press Ctrl+R

4. **Upload**: Click the → button or press Ctrl+U
   - Wait for "Done uploading" message
   - The ESP32 will restart automatically

5. **Open Serial Monitor**: 
   - Click the magnifying glass icon or press Ctrl+Shift+M
   - Set baud rate to **115200**
   - You should see:
     ```
     === Drawing Robot WiFi Controller ===
     Core 0: Stepper Motor Control (time-sensitive)
     Core 1: WiFi, Pen Control, OTA Updates
     WiFi AP started
     AP IP address: 192.168.4.1
     ```

### Subsequent Uploads (Over WiFi - OTA)

After the first USB upload, you can update firmware wirelessly:

**Method 1: Using ElegantOTA Web Interface**
1. Connect to robot's WiFi (or same network if connected to existing WiFi)
2. Open browser: `http://192.168.4.1/update` (or robot's IP)
3. Enter OTA password: `draw1234`
4. Click "Choose File" and select your compiled `.bin` file
5. Click "Update"
6. Wait for upload and automatic reboot

**Method 2: Using Arduino IDE**
1. In Arduino IDE, go to **Tools → Port**
2. Select the network port: `DrawingRobot at 192.168.4.1` (or your robot's IP)
3. Upload normally - it will upload over WiFi

---

## Python Client Setup

### Step 1: Install Python
Download Python 3.7+ from: https://www.python.org/downloads/

### Step 2: Install Dependencies
Open terminal/command prompt in the `UI` folder and run:

```bash
pip install -r requirements.txt
```

This installs:
- `requests` - HTTP client
- `numpy` - Numerical operations
- `Pillow` - Image processing
- `opencv-python` - Computer vision (edge detection)

### Step 3: Verify Installation
```bash
python image_to_vectors.py --help
```

---

## Using the System

### Basic Workflow

1. **Power on the robot** and wait for WiFi to start
2. **Connect to robot's network**:
   - If AP mode: Connect to "DrawingRobot" network (password: `draw1234`)
   - If connected to WiFi: Connect to the same network
3. **Find robot's IP address**:
   - Check Serial Monitor output
   - Or visit `http://192.168.4.1/` (default AP IP)
4. **Convert and send image**:
   ```bash
   python image_to_vectors.py your_image.png --ip 192.168.4.1
   ```

### Example Commands

**Basic usage:**
```bash
python image_to_vectors.py image.png
```

**With custom IP:**
```bash
python image_to_vectors.py image.png --ip 192.168.1.100
```

**Adjust edge detection:**
```bash
python image_to_vectors.py image.png --low-threshold 30 --high-threshold 100
```

**Use threshold method instead of Canny:**
```bash
python image_to_vectors.py image.png --method threshold
```

**Save vectors without sending:**
```bash
python image_to_vectors.py image.png --no-send --save-json vectors.json
```

### Web Interface

1. Open browser: `http://192.168.4.1/` (or robot's IP)
2. You'll see:
   - Connection status
   - Drawing interface
   - Link to OTA update page
3. You can manually send JSON paths:
   ```json
   {
     "paths": [
       [[100, 100], [200, 100], [200, 200], [100, 200], [100, 100]]
     ]
   }
   ```

### API Endpoints

- `GET /` - Web interface
- `POST /draw` - Send drawing paths (JSON format)
- `GET /status` - Get robot status and queue sizes
- `GET /update` - OTA update interface

---

## Troubleshooting

### Arduino IDE Issues

**"Board not found"**
- Install ESP32 board support (see Arduino IDE Setup)
- Check USB cable connection
- Try different USB port

**"Upload failed"**
- Hold BOOT button while clicking Upload
- Lower upload speed in Tools → Upload Speed
- Check COM port selection

**"Library not found"**
- Install libraries from Library Manager
- Check library names match exactly

### WiFi Connection Issues

**Can't connect to robot WiFi**
- Check SSID: "DrawingRobot"
- Password: "draw1234"
- Wait 30 seconds after robot starts
- Try forgetting network and reconnecting

**Can't find robot IP**
- Check Serial Monitor output
- Default AP IP: 192.168.4.1
- If connected to WiFi, check router's device list

**Connection timeout**
- Ensure you're on the same network
- Check firewall settings
- Verify IP address is correct

### Python Client Issues

**"Module not found"**
- Run: `pip install -r requirements.txt`
- Use `pip3` instead of `pip` on Linux/Mac

**"Connection refused"**
- Check robot is powered on
- Verify IP address
- Check robot's Serial Monitor for errors

**"Image processing fails"**
- Ensure image file exists
- Check image format (PNG, JPG supported)
- Try smaller image size

### Drawing Issues

**Robot doesn't move**
- Check stepper motor connections
- Verify power supply
- Check Serial Monitor for errors
- Test with simple coordinates via web interface

**Drawing is inaccurate**
- Calibrate belt lengths (L1, L2 in code)
- Check pulley diameter setting
- Verify BASE distance between motors

**Pen doesn't lift/lower**
- Check servo connection to pin 19
- Verify servo angles (PEN_UP_ANGLE, PEN_DOWN_ANGLE)
- Test servo separately

---

## Quick Reference

### Important Files
- `Arduino code/drawing/drawing_wifi.ino` - Main ESP32 code
- `UI/image_to_vectors.py` - Python client for image conversion
- `UI/requirements.txt` - Python dependencies

### Default Settings
- **AP SSID**: DrawingRobot
- **AP Password**: draw1234
- **OTA Password**: draw1234
- **Default AP IP**: 192.168.4.1
- **Drawing Area**: 50-650mm (X), 50-950mm (Y)

### Pin Assignments
- **Left Stepper**: Step=17, Dir=16
- **Right Stepper**: Step=5, Dir=18
- **Servo (Pen)**: Pin 19

---

## Next Steps

1. **Test basic movement**: Send simple paths via web interface
2. **Calibrate robot**: Adjust L1, L2 belt lengths if needed
3. **Tune parameters**: Adjust stepper speeds, accelerations
4. **Experiment**: Try different images and edge detection settings

For more details, see `UI/README_WIFI.md`

