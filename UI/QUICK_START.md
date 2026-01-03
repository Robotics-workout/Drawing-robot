# Quick Start Guide

## 🚀 Getting Started in 5 Minutes

### Step 1: Setup Arduino Code
1. Open `Arduino code/drawing/drawing_wifi.ino` in Arduino IDE
2. Install required libraries (see SETUP_GUIDE.md)
3. Upload to ESP32 via USB
4. Open Serial Monitor (115200 baud) to see IP address

### Step 2: Connect to Robot
- **AP Mode**: Connect to "DrawingRobot" WiFi (password: `draw1234`)
- **WiFi Mode**: Connect to same network as robot (check Serial Monitor for IP)

### Step 3: Send Your First Drawing

**Option A: From an Image**
```bash
python image_to_vectors.py your_image.png
```

**Option B: Send Waypoints**
```bash
python send_waypoints.py --waypoints "100,100 200,100 200,200 100,200 100,100"
```

That's it! The robot will start drawing.

---

## 📚 Common Commands

### Image to Vectors
```bash
# Basic usage
python image_to_vectors.py image.png

# Adjust edge detection
python image_to_vectors.py image.png --low-threshold 30 --high-threshold 100

# Save vectors without sending
python image_to_vectors.py image.png --save-json vectors.json --no-send
```

### Send Waypoints
```bash
# From command line
python send_waypoints.py --waypoints "100,100 200,100 200,200"

# From JSON file
python send_waypoints.py waypoints.json

# Without scaling
python send_waypoints.py waypoints.json --no-scale
```

### Examples and Tutorials
```bash
# Run interactive examples
python examples.py

# Run specific example
python examples.py 1
```

---

## 🎯 Quick Examples

### Draw a Square
```bash
python send_waypoints.py --waypoints "100,100 200,100 200,200 100,200 100,100"
```

### Draw a Triangle
```bash
python send_waypoints.py --waypoints "300,300 400,300 350,400 300,300"
```

### Convert Logo to Drawing
```bash
python image_to_vectors.py logo.png --method threshold
```

---

## 🔧 Troubleshooting

**Can't connect?**
- Check robot IP in Serial Monitor
- Verify WiFi connection
- Try: `python send_waypoints.py --ip 192.168.1.100`

**Drawing too detailed?**
- Add `--epsilon 2.0` to simplify
- Add `--min-length 20` to filter small paths

**Need help?**
- Run `python examples.py` for tutorials
- See `SETUP_GUIDE.md` for detailed setup
- Check `README_WIFI.md` for full documentation

---

## 📖 More Information

- **Full Setup**: See `SETUP_GUIDE.md`
- **API Documentation**: See `README_WIFI.md`
- **Examples**: Run `python examples.py`
- **Waypoint Format**: See examples in `examples.py`

