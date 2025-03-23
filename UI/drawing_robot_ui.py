import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue
import threading
import time

# Robot parameters
BASE = 700.0  # Distance between motors (mm)
PULLEY_DIAMETER = 12.7  # Pulley diameter in mm
MOTOR_STEPS = 200  # Steps per revolution
MICROSTEPS = 1  # Microstepping level
STEPS_PER_REV = MOTOR_STEPS * MICROSTEPS
MM_PER_STEP = (np.pi * PULLEY_DIAMETER) / STEPS_PER_REV  # Steps per mm movement
L_ARM = 124.959  # Length of the robot's arm
DRAWING_AREA_HEIGHT = 1000.0  # Height of the drawing area (mm)
DRAWING_AREA_X_BIAS = 50  # Bias to keep space around edges for safety
DRAWING_AREA_Y_BIAS = 50  # Bias to keep space around edges for safety

drawing_area_x_limits = [DRAWING_AREA_X_BIAS, BASE - DRAWING_AREA_X_BIAS]
drawing_area_y_limits = [DRAWING_AREA_Y_BIAS, DRAWING_AREA_HEIGHT - DRAWING_AREA_Y_BIAS]

# Queue to store target positions
target_queue = Queue()

# Initialize figure
fig, ax = plt.subplots()
ax.set_xlim(-50, BASE + 50)  # Set X-axis limits
ax.set_ylim(DRAWING_AREA_HEIGHT + 50, -50)  # Set Y-axis limits (inverted for top-left origin)
ax.set_aspect('equal')  # Ensure the plot has equal scaling for X and Y axes
ax.set_title("Drawing Robot Simulation")

# Plot elements
pen, = ax.plot([], [], 'ro', markersize=8)  # Pen position (red dot)
left_motor, = ax.plot([0], [0], 'bo', markersize=10)  # Left motor position (blue dot)
right_motor, = ax.plot([BASE], [0], 'bo', markersize=10)  # Right motor position (blue dot)
line, = ax.plot([], [], 'r-', lw=2)  # Line tracing (blue line)
left_belt_line, = ax.plot([], [], 'b-')  # Belt connecting left motor to pen
right_belt_line, = ax.plot([], [], 'b-')  # Belt connecting right motor to pen

# Store previous position of the pen to trace the path
pen_positions_x = []  # List to store x coordinates of the pen path
pen_positions_y = []  # List to store y coordinates of the pen path

# Initialize text annotations for Z1 and Z2 lengths
z1_text = ax.text(0, 0, '', color='green', fontsize=5)  # Annotation for Z1
z2_text = ax.text(0, 0, '', color='orange', fontsize=5)  # Annotation for Z2

# Function to calculate belt lengths
def calculate_belt_lengths(x, y):
    y = DRAWING_AREA_HEIGHT - y  # Convert y-coordinate to match top-left origin
    Z1 = np.sqrt(x**2 + y**2)
    Z2 = np.sqrt((BASE - x)**2 + y**2)
    return Z1, Z2

# Function to calculate XY position from belt lengths
def calculate_xy(Z1, Z2):
    x = (BASE**2 + Z1**2 - Z2**2) / (2 * BASE)  # Use law of cosines
    y = np.sqrt(Z1**2 - x**2)  # Use Pythagoras' theorem to calculate y
    y = DRAWING_AREA_HEIGHT - y  # Convert y back to top-left origin
    return x, y

# Animation update function
def update(frame):
    global previous_position

    if not target_queue.empty():
        target_x, target_y = target_queue.get()  # Get next target position from the queue
        
        # Calculate the belt lengths using inverse kinematics
        Z1, Z2 = calculate_belt_lengths(target_x, target_y)
        
        # Calculate the position based on the belt lengths (for visualization)
        x, y = calculate_xy(Z1, Z2)

        # Update pen position
        pen.set_data([x], [y])

        # Append the current position to the pen positions list
        pen_positions_x.append(x)
        pen_positions_y.append(y)

        # Update the line with the full trace
        line.set_data(pen_positions_x, pen_positions_y)

        # Update text annotations for Z1 and Z2 lengths
        z1_text.set_text(f'Z1: {Z1:.2f} mm')
        z2_text.set_text(f'Z2: {Z2:.2f} mm')

        # Position the text annotations near the motors
        z1_text.set_position((x/2, y/2))  # Z1 near the left motor
        z2_text.set_position(((BASE+x)/2, y/2))  # Z2 near the right motor

        left_belt_line.set_data([0, x], [0, y])
        right_belt_line.set_data([BASE, x], [0, y])
    return pen, line, left_belt_line, right_belt_line, z1_text, z2_text

# Background thread to simulate drawing movements
def simulation_thread():
    while True:
        for x in np.linspace(drawing_area_x_limits[0], drawing_area_x_limits[1], 20):  # Move along x-axis
            y = 100 + 50 * np.sin(x * np.pi / 250)  # Sinusoidal y-motion
            target_queue.put((x, y))  # Put target position into the queue
            time.sleep(0.1)  # Delay to simulate time between movements

# Start background thread
threading.Thread(target=simulation_thread, daemon=True).start()

# Start animation
ani = animation.FuncAnimation(fig, update, interval=100, blit=True)
plt.show()
