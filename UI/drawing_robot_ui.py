import numpy as np
from math import cos, sin, atan2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue
import threading
import time
from matplotlib.patches import Rectangle, Circle, Polygon

# Robot parameters
BASE = 700.0  # Distance between motors (mm)
PULLEY_DIAMETER = 12.7  # Pulley diameter in mm
MOTOR_STEPS = 200  # Steps per revolution
MICROSTEPS = 1  # Microstepping level
STEPS_PER_REV = MOTOR_STEPS * MICROSTEPS
MM_PER_STEP = (np.pi * PULLEY_DIAMETER) / STEPS_PER_REV  # Steps per mm movement
L_ARM = 124.959  # Length of the robot's arm
TOTAL_BELT_LENGTH = 1800  # Total length of the belt (mm)
DRAWING_AREA_HEIGHT = 1500.0  # Height of the drawing area (mm)
DRAWING_AREA_X_BIAS = 50  # Bias to keep space around edges for safety
DRAWING_AREA_Y_BIAS = 50  # Bias to keep space around edges for safety

drawing_area_x_limits = [DRAWING_AREA_X_BIAS, BASE - DRAWING_AREA_X_BIAS]
drawing_area_y_limits = [DRAWING_AREA_Y_BIAS, DRAWING_AREA_HEIGHT - DRAWING_AREA_Y_BIAS]

target_queue = Queue()  # Queue to store target positions

# Initialize figure
fig, ax = plt.subplots(figsize=(16, 9))
ax.set_xlim(-DRAWING_AREA_X_BIAS, BASE + DRAWING_AREA_X_BIAS)
ax.set_ylim(DRAWING_AREA_HEIGHT + DRAWING_AREA_Y_BIAS, -DRAWING_AREA_Y_BIAS)
ax.set_aspect("equal")
ax.set_title("Drawing Robot Simulation")

# Gondola parameters
GONDOLA_OUTER_RADIUS = 50
GONDOLA_INNER_RADIUS = 8
WING_LENGTH = 100

gondola_center = [(drawing_area_x_limits[0] + drawing_area_x_limits[1]) / 2, 
                  (drawing_area_y_limits[0] + drawing_area_y_limits[1]) / 2]
gondola_black_circle = Circle(gondola_center, GONDOLA_OUTER_RADIUS, color="black", zorder=23)
gondola_red_circle = Circle(gondola_center, GONDOLA_INNER_RADIUS, color="red", zorder=24)
# Initial wing angles
right_wing_angle = 0
left_wing_angle = 0
gondola_right_wing = Polygon(
    [
        [gondola_center[0] - GONDOLA_OUTER_RADIUS * sin(right_wing_angle), gondola_center[1] - GONDOLA_OUTER_RADIUS * cos(right_wing_angle)],
        [gondola_center[0] + WING_LENGTH * cos(right_wing_angle), gondola_center[1] - WING_LENGTH * sin(right_wing_angle)],
        [gondola_center[0] + GONDOLA_OUTER_RADIUS * sin(right_wing_angle), gondola_center[1] + GONDOLA_OUTER_RADIUS * cos(right_wing_angle)],
    ],
    closed=True, facecolor="grey", edgecolor="black", zorder=22,
)
gondola_left_wing = Polygon(
    [
        [gondola_center[0] - GONDOLA_OUTER_RADIUS * sin(left_wing_angle), gondola_center[1] - GONDOLA_OUTER_RADIUS * cos(left_wing_angle)],
        [gondola_center[0] + WING_LENGTH * cos(left_wing_angle), gondola_center[1] - WING_LENGTH * sin(left_wing_angle)],
        [gondola_center[0] + GONDOLA_OUTER_RADIUS * sin(left_wing_angle), gondola_center[1] + GONDOLA_OUTER_RADIUS * cos(left_wing_angle)],
    ],
    closed=True, facecolor="grey", edgecolor="black", zorder=21,
)
ax.add_patch(gondola_black_circle)
ax.add_patch(gondola_red_circle)
ax.add_patch(gondola_right_wing)
ax.add_patch(gondola_left_wing)

gondola_position_text = ax.text(0, 0, "", color="red", fontsize=5)

gondola_positions_x = []  # X coordinates of gondola path
gondola_positions_y = []  # Y coordinates of gondola path
(line,) = ax.plot([], [], "r-", lw=2)

# Transit path
transit_path_x = []
transit_path_y = []
(transit_line,) = ax.plot([], [], 'r--', lw=2)

# Motor visualization
MOTOR_SIZE = 80
motor_center_left = (0, 0)
motor_center_right = (BASE, 0)
left_motor_square = Rectangle((motor_center_left[0] - MOTOR_SIZE / 2, motor_center_left[1] - MOTOR_SIZE / 2), MOTOR_SIZE, MOTOR_SIZE, color="black", zorder=10)
left_motor_grey_circle = Circle(motor_center_left, MOTOR_SIZE * 0.35, color="grey", zorder=11)
left_motor_black_circle = Circle(motor_center_left, MOTOR_SIZE * 0.1, color="black", zorder=12)
right_motor_square = Rectangle((motor_center_right[0] - MOTOR_SIZE / 2, motor_center_right[1] - MOTOR_SIZE / 2), MOTOR_SIZE, MOTOR_SIZE, color="black", zorder=10)
right_motor_grey_circle = Circle(motor_center_right, MOTOR_SIZE * 0.35, color="grey", zorder=11)
right_motor_black_circle = Circle(motor_center_right, MOTOR_SIZE * 0.1, color="black", zorder=12)
for patch in [left_motor_square, left_motor_grey_circle, left_motor_black_circle, right_motor_square, right_motor_grey_circle, right_motor_black_circle]:
    ax.add_patch(patch)

# Belt lines
left_belt_line, = ax.plot([], [], color="black", linestyle="--")
left_belt_slack_line, = ax.plot([], [], color="black", linestyle="--")
right_belt_line, = ax.plot([], [], color="black", linestyle="--")
right_belt_slack_line, = ax.plot([], [], color="black", linestyle="--")

# Text annotations for belt lengths
z1_text = ax.text(0, 0, "", color="green", fontsize=5)
z2_text = ax.text(0, 0, "", color="orange", fontsize=5)

# --- Utility Functions ---
def calculate_belt_lengths(x, y):
    Z1 = np.sqrt(x ** 2 + y ** 2)
    Z1_slack = abs(TOTAL_BELT_LENGTH - Z1)
    Z2 = np.sqrt((BASE - x) ** 2 + y ** 2)
    Z2_slack = abs(TOTAL_BELT_LENGTH - Z2)
    # This is where stepper motors would be controlled in a real robot
    return Z1, Z2, Z1_slack, Z2_slack

def calculate_xy(Z1, Z2):
    x = (BASE ** 2 + Z1 ** 2 - Z2 ** 2) / (2 * BASE)
    y = np.sqrt(Z1 ** 2 - x ** 2)
    return x, y

# --- Animation Update ---
def update(frame):
    if not target_queue.empty():
        target_x, target_y, path_detail = target_queue.get()

        # Calculate belt lengths and gondola position
        Z1, Z2, Z1_slack, Z2_slack = calculate_belt_lengths(target_x, target_y)
        x, y = calculate_xy(Z1, Z2) # just used to cross verify the target position

        # Calculate wing angles for gondola visualization
        right_wing_angle = np.pi - atan2(target_y, target_x - BASE)
        left_wing_angle = atan2(target_y, target_x)

        # Update gondola (center and wings) position
        gondola_black_circle.center = (x, y)
        gondola_red_circle.center = (x, y)
        gondola_right_wing.set_xy([
            [x - GONDOLA_OUTER_RADIUS * sin(right_wing_angle), y - GONDOLA_OUTER_RADIUS * cos(right_wing_angle)],
            [x + WING_LENGTH * cos(right_wing_angle), y - WING_LENGTH * sin(right_wing_angle)],
            [x + GONDOLA_OUTER_RADIUS * sin(right_wing_angle), y + GONDOLA_OUTER_RADIUS * cos(right_wing_angle)],
        ])
        gondola_left_wing.set_xy([
            [x + GONDOLA_OUTER_RADIUS * sin(left_wing_angle), y - GONDOLA_OUTER_RADIUS * cos(left_wing_angle)],
            [x - WING_LENGTH * cos(left_wing_angle), y - WING_LENGTH * sin(left_wing_angle)],
            [x - GONDOLA_OUTER_RADIUS * sin(left_wing_angle), y + GONDOLA_OUTER_RADIUS * cos(left_wing_angle)],
        ])

        # Store gondola path for trace
        if path_detail == 1:
            # If it's a drawing path, add to gondola positions
            gondola_positions_x.append(x)
            gondola_positions_y.append(y)
            line.set_data(gondola_positions_x, gondola_positions_y)
        else:
            # If it's a transit path, add to transit path
            transit_path_x.append(x)
            transit_path_y.append(y)
            transit_line.set_data(transit_path_x, transit_path_y)

        # Update belt length text annotations
        z1_text.set_text(f"Z1: {Z1:.2f} mm")
        z2_text.set_text(f"Z2: {Z2:.2f} mm")

        # Update gondola position annotation
        gondola_position_text.set_position((x, y + GONDOLA_OUTER_RADIUS + 20))
        gondola_position_text.set_text(f"{x:.2f}, {y:.2f}")

        # Position belt length text near motors
        z1_text.set_position((x / 2, y / 2))
        z2_text.set_position(((BASE + x) / 2, y / 2))

        # Update belt and slack lines
        left_belt_line.set_data([0, x], [0, y])
        right_belt_line.set_data([BASE, x], [0, y])
        left_belt_slack_line.set_data([motor_center_left[0], motor_center_left[0]], [motor_center_left[1], Z1_slack])
        right_belt_slack_line.set_data([motor_center_right[0], motor_center_right[0]], [motor_center_right[1], Z2_slack])
    return (
        gondola_black_circle, gondola_red_circle, line, left_belt_line, right_belt_line,
        z1_text, z2_text, left_belt_slack_line, right_belt_slack_line, gondola_position_text,
        gondola_right_wing, gondola_left_wing, transit_line
    )

# --- Simulation Thread ---
def simulation_thread():
    """Simulates the drawing robot by generating target positions."""
    # Generate a square path within the drawing area
    margin = 100
    x0 = drawing_area_x_limits[0] + margin
    x1 = drawing_area_x_limits[1] - margin
    y0 = drawing_area_y_limits[0] + margin
    y1 = drawing_area_y_limits[1] - margin
    square_points = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]

    # Add points to travel from current position to the first point
    x_start, y_start = gondola_center
    x_end, y_end = square_points[0]
    for t in np.linspace(0, 1, 40):
        x = x_start + (x_end - x_start) * t
        y = y_start + (y_end - y_start) * t
        target_queue.put((x, y, 0)) # x, y and flag to indicate if its transit or drawing, 0: transit, 1: drawing
        time.sleep(0.05)

    for i in range(len(square_points) - 1):
        x_start, y_start = square_points[i]
        x_end, y_end = square_points[i + 1]
        for t in np.linspace(0, 1, 40):
            x = x_start + (x_end - x_start) * t
            y = y_start + (y_end - y_start) * t
            target_queue.put((x, y, 1)) # x, y and flag to indicate if its transit or drawing, 0: transit, 1: drawing
            time.sleep(0.05)

threading.Thread(target=simulation_thread, daemon=True).start()
ani = animation.FuncAnimation(fig, update, interval=100, blit=True)
plt.show()
