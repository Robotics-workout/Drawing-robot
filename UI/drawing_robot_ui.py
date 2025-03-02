import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import simpledialog

# Robot arm lengths
L1 = 100  # Length of arm 1
L2 = 100  # Length of arm 2

# Initialize target point
target_x, target_y = 50, 50

# Function to compute inverse kinematics
def inverse_kinematics(x, y):
    distance = np.sqrt(x**2 + y**2)
    if distance > (L1 + L2) or distance < abs(L1 - L2):
        return None, None  # Out of reach

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))

    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

# Function to update robot arm position
def update_arm(i):
    global target_x, target_y
    theta1, theta2 = inverse_kinematics(target_x, target_y)
    
    if theta1 is None or theta2 is None:
        return  # Skip update if out of reach

    # Compute joint positions
    x1, y1 = L1 * np.cos(theta1), L1 * np.sin(theta1)
    x2, y2 = x1 + L2 * np.cos(theta1 + theta2), y1 + L2 * np.sin(theta1 + theta2)

    # Update plot
    arm.set_data([0, x1, x2], [0, y1, y2])
    pen.set_data(x2, y2)

# Function to update target position
def set_target():
    global target_x, target_y
    new_x = simpledialog.askfloat("Input", "Enter X coordinate:", minvalue=-200, maxvalue=200)
    new_y = simpledialog.askfloat("Input", "Enter Y coordinate:", minvalue=-200, maxvalue=200)
    if new_x is not None and new_y is not None:
        target_x, target_y = new_x, new_y

# Create UI Window
root = tk.Tk()
root.title("Drawing Robot Simulator")
tk.Button(root, text="Move to Position", command=set_target).pack()

# Create figure for visualization
fig, ax = plt.subplots()
ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_aspect('equal')
ax.grid(True)

# Create robot arm plot elements
arm, = ax.plot([], [], 'ro-', markersize=8, linewidth=3)  # Robot arms
pen, = ax.plot([], [], 'bo', markersize=10)  # Pen tip

# Animate the robot arm
ani = FuncAnimation(fig, update_arm, frames=100, interval=50)

# Run UI and Visualization
tk.Button(root, text="Show Simulation", command=lambda: plt.show()).pack()
root.mainloop()
