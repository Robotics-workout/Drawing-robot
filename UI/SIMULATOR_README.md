# Drawing Robot Simulator

Visual simulation UI to see the robot drawing vector paths in real-time.

## Features

- 🎨 **Visual Simulation**: See the robot's gondola moving and drawing paths
- 📊 **Real-time Animation**: Watch paths being drawn as the robot moves
- 🔌 **Robot Integration**: Connect to robot and send paths while visualizing
- 📁 **File Support**: Load paths from JSON files or convert images
- 🎯 **Path Visualization**: See drawing paths (pen down) vs transit paths (pen up)
- 📐 **Belt Visualization**: See belt lengths and robot kinematics

## Quick Start

### Basic Simulation

**Simulate from JSON file:**
```bash
python drawing_simulator.py --file vectors.json
```

**Simulate from image:**
```bash
python drawing_simulator.py --image logo.png
```

### Connect to Robot

**Send paths to robot and visualize:**
```bash
python drawing_simulator.py --file vectors.json --robot 192.168.4.1 --send
```

**Just visualize (don't send):**
```bash
python drawing_simulator.py --file vectors.json --robot 192.168.4.1
```

## Usage Examples

### Example 1: Simulate a Drawing
```bash
# First, convert an image to vectors
python image_to_vectors.py logo.png --save-json logo_vectors.json --no-send

# Then simulate it
python drawing_simulator.py --file logo_vectors.json
```

### Example 2: Send and Visualize
```bash
# Convert image and send to robot, while visualizing
python drawing_simulator.py --image logo.png --robot 192.168.4.1 --send
```

### Example 3: Test Waypoints
```bash
# Create a simple waypoint file
echo '{"paths":[[[100,100],[200,100],[200,200],[100,200],[100,100]]]}' > square.json

# Simulate it
python drawing_simulator.py --file square.json
```

## What You'll See

The simulator displays:

1. **Drawing Board**: The drawing area with boundaries
2. **Motors**: Left and right stepper motors at the top
3. **Gondola**: The robot's drawing head (black circle with red center)
4. **Wings**: Gondola orientation showing belt angles
5. **Belts**: Dashed lines showing belt connections
6. **Drawing Paths**: Blue lines showing where the pen is drawing
7. **Transit Paths**: Red dashed lines showing pen-up movements
8. **Status Info**: Current position, pen state, belt lengths

## Controls

- **Close Window**: Close the matplotlib window to exit
- **Animation**: Automatically updates as paths are processed
- **Real-time**: Updates every 50ms for smooth animation

## Integration with Robot

When connected to a robot (`--robot` flag):

- **Monitor Mode**: Visualizes what the robot is drawing
- **Send Mode** (`--send`): Sends paths to robot and visualizes simultaneously
- **Status Updates**: Shows robot connection status

## Tips

1. **Large Paths**: For complex drawings, the animation may take time to complete
2. **Performance**: Close other applications if animation is slow
3. **Scaling**: Paths are automatically scaled to fit the drawing area
4. **Preview**: Use simulation before sending to robot to verify paths

## Troubleshooting

**"No module named 'matplotlib'"**
```bash
pip install matplotlib
```

**Animation is slow**
- Reduce the number of points in your paths
- Close other applications
- Use smaller images

**Robot connection fails**
- Check robot IP address
- Ensure robot is powered on and connected to WiFi
- Verify you're on the same network

## Advanced Usage

### Programmatic Usage

```python
from drawing_simulator import DrawingSimulator

# Create simulator
simulator = DrawingSimulator(robot_ip="192.168.4.1")

# Load paths
paths = [[(100, 100), (200, 100), (200, 200), (100, 200), (100, 100)]]

# Add to simulator
simulator.add_path(paths, send_to_robot=True)

# Start animation
simulator.start_animation()
```

### Custom Paths

```python
from drawing_simulator import DrawingSimulator

simulator = DrawingSimulator()

# Create custom paths
square = [(100, 100), (200, 100), (200, 200), (100, 200), (100, 100)]
triangle = [(300, 300), (400, 300), (350, 400), (300, 300)]

simulator.add_path([square, triangle])
simulator.start_animation()
```

## See Also

- `image_to_vectors.py` - Convert images to vectors
- `send_waypoints.py` - Send waypoints to robot
- `examples.py` - More usage examples

