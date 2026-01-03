"""
Drawing Robot Simulation UI
Visualizes the robot drawing vector paths in real-time.
Can connect to robot or simulate from vector files.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle, Polygon
from math import cos, sin, atan2
import json
import requests
import threading
import time
from queue import Queue
from typing import List, Tuple, Optional
import argparse
from image_to_vectors import DrawingRobotClient, scale_paths_to_fit


# Robot parameters (must match Arduino code)
BASE = 700.0  # Distance between motors (mm)
DRAWING_AREA_HEIGHT = 1000.0
DRAWING_AREA_X_BIAS = 50
DRAWING_AREA_Y_BIAS = 50
L_ARM = 124.959

drawing_area_x_limits = [DRAWING_AREA_X_BIAS, BASE - DRAWING_AREA_X_BIAS]
drawing_area_y_limits = [DRAWING_AREA_Y_BIAS, DRAWING_AREA_HEIGHT - DRAWING_AREA_Y_BIAS]

# Gondola visualization parameters
GONDOLA_OUTER_RADIUS = 50
GONDOLA_INNER_RADIUS = 8
WING_LENGTH = 100
MOTOR_SIZE = 80


class DrawingSimulator:
    """Simulates the drawing robot and visualizes its movement."""

    def __init__(self, robot_ip: Optional[str] = None, animation_delay: float = 0.1):
        """
        Initialize the simulator.

        Args:
            robot_ip: IP address of robot (None for simulation only)
            animation_delay: Delay between points in seconds (default: 0.1s = 100ms)
        """
        self.robot_ip = robot_ip
        self.client = DrawingRobotClient(robot_ip) if robot_ip else None
        self.animation_delay = animation_delay

        # Current position
        self.current_x = (drawing_area_x_limits[0] + drawing_area_x_limits[1]) / 2
        self.current_y = (drawing_area_y_limits[0] + drawing_area_y_limits[1]) / 2
        self.pen_down = False

        # Path queue for animation
        self.path_queue = Queue()
        self.drawing_paths = []  # Completed drawing paths
        self.transit_paths = []  # Transit paths (pen up)
        self.last_update_time = time.time()

        # Setup matplotlib
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.setup_plot()

        # Animation
        self.ani = None
        self.is_animating = False

    def setup_plot(self):
        """Setup the matplotlib plot with robot visualization."""
        self.ax.set_xlim(-DRAWING_AREA_X_BIAS, BASE + DRAWING_AREA_X_BIAS)
        self.ax.set_ylim(
            DRAWING_AREA_HEIGHT + DRAWING_AREA_Y_BIAS, -DRAWING_AREA_Y_BIAS
        )
        self.ax.set_aspect("equal")
        self.ax.set_title("Drawing Robot Simulation", fontsize=14, fontweight="bold")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")

        # Draw drawing area border
        border = Rectangle(
            (drawing_area_x_limits[0], drawing_area_y_limits[0]),
            drawing_area_x_limits[1] - drawing_area_x_limits[0],
            drawing_area_y_limits[1] - drawing_area_y_limits[0],
            fill=False,
            edgecolor="blue",
            linewidth=2,
            linestyle="--",
            zorder=1,
        )
        self.ax.add_patch(border)

        # Draw motors
        motor_center_left = (0, 0)
        motor_center_right = (BASE, 0)

        left_motor = Rectangle(
            (
                motor_center_left[0] - MOTOR_SIZE / 2,
                motor_center_left[1] - MOTOR_SIZE / 2,
            ),
            MOTOR_SIZE,
            MOTOR_SIZE,
            color="black",
            zorder=10,
        )
        right_motor = Rectangle(
            (
                motor_center_right[0] - MOTOR_SIZE / 2,
                motor_center_right[1] - MOTOR_SIZE / 2,
            ),
            MOTOR_SIZE,
            MOTOR_SIZE,
            color="black",
            zorder=10,
        )
        self.ax.add_patch(left_motor)
        self.ax.add_patch(right_motor)

        # Motor labels
        self.ax.text(
            motor_center_left[0],
            motor_center_left[1] - 60,
            "Left Motor",
            ha="center",
            fontsize=10,
            fontweight="bold",
        )
        self.ax.text(
            motor_center_right[0],
            motor_center_right[1] - 60,
            "Right Motor",
            ha="center",
            fontsize=10,
            fontweight="bold",
        )

        # Gondola (will be updated in animation)
        self.gondola_black = Circle(
            (self.current_x, self.current_y),
            GONDOLA_OUTER_RADIUS,
            color="black",
            zorder=23,
        )
        self.gondola_red = Circle(
            (self.current_x, self.current_y),
            GONDOLA_INNER_RADIUS,
            color="red",
            zorder=24,
        )
        self.gondola_left_wing = Polygon(
            [(0, 0), (0, 0), (0, 0)],
            closed=True,
            facecolor="grey",
            edgecolor="black",
            zorder=22,
        )
        self.gondola_right_wing = Polygon(
            [(0, 0), (0, 0), (0, 0)],
            closed=True,
            facecolor="grey",
            edgecolor="black",
            zorder=22,
        )

        self.ax.add_patch(self.gondola_black)
        self.ax.add_patch(self.gondola_red)
        self.ax.add_patch(self.gondola_left_wing)
        self.ax.add_patch(self.gondola_right_wing)

        # Belt lines
        (self.left_belt_line,) = self.ax.plot(
            [], [], color="black", linestyle="--", linewidth=1, zorder=5
        )
        (self.right_belt_line,) = self.ax.plot(
            [], [], color="black", linestyle="--", linewidth=1, zorder=5
        )

        # Drawing paths (will be drawn as robot moves)
        self.drawing_lines = []  # List of line objects for each path
        (self.transit_line,) = self.ax.plot(
            [], [], "r--", linewidth=1, alpha=0.5, label="Transit (pen up)"
        )
        (self.current_drawing_line,) = self.ax.plot(
            [], [], "b-", linewidth=2, label="Drawing (pen down)"
        )

        # Status text (positioned at bottom)
        self.status_text = self.ax.text(
            0.02,
            0.02,
            "",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment="bottom",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

        # Legend (positioned to avoid motors at top)
        self.ax.legend(loc="lower right", framealpha=0.9)

    def calculate_belt_lengths(self, x, y):
        """Calculate belt lengths from position."""
        Z1 = np.sqrt(x**2 + y**2)
        Z2 = np.sqrt((BASE - x) ** 2 + y**2)
        return Z1, Z2

    def update_gondola(self, x, y):
        """Update gondola position and orientation."""
        # Calculate wing angles
        right_wing_angle = np.pi - atan2(y, x - BASE)
        left_wing_angle = atan2(y, x)

        # Update gondola circles
        self.gondola_black.center = (x, y)
        self.gondola_red.center = (x, y)

        # Update wings
        self.gondola_right_wing.set_xy(
            [
                [
                    x - GONDOLA_OUTER_RADIUS * sin(right_wing_angle),
                    y - GONDOLA_OUTER_RADIUS * cos(right_wing_angle),
                ],
                [
                    x + WING_LENGTH * cos(right_wing_angle),
                    y - WING_LENGTH * sin(right_wing_angle),
                ],
                [
                    x + GONDOLA_OUTER_RADIUS * sin(right_wing_angle),
                    y + GONDOLA_OUTER_RADIUS * cos(right_wing_angle),
                ],
            ]
        )
        self.gondola_left_wing.set_xy(
            [
                [
                    x + GONDOLA_OUTER_RADIUS * sin(left_wing_angle),
                    y - GONDOLA_OUTER_RADIUS * cos(left_wing_angle),
                ],
                [
                    x - WING_LENGTH * cos(left_wing_angle),
                    y - WING_LENGTH * sin(left_wing_angle),
                ],
                [
                    x - GONDOLA_OUTER_RADIUS * sin(left_wing_angle),
                    y + GONDOLA_OUTER_RADIUS * cos(left_wing_angle),
                ],
            ]
        )

        # Update belt lines
        self.left_belt_line.set_data([0, x], [0, y])
        self.right_belt_line.set_data([BASE, x], [0, y])

    def add_path(
        self,
        paths: List[List[Tuple[float, float]]],
        send_to_robot: bool = False,
        scale_to_fit: bool = True,
    ):
        """
        Add paths to the simulation queue.

        Args:
            paths: List of paths to draw
            send_to_robot: If True and robot_ip is set, send to robot
            scale_to_fit: If True, scale paths to fit within drawing area
        """
        if scale_to_fit:
            # Scale paths to fit within drawing area
            target_width = drawing_area_x_limits[1] - drawing_area_x_limits[0]
            target_height = drawing_area_y_limits[1] - drawing_area_y_limits[0]

            # Scale paths (this centers them at target_width/2, target_height/2 relative to origin)
            scaled_paths = scale_paths_to_fit(
                paths,
                target_width,
                target_height,
            )

            # Offset scaled paths to actual drawing area position
            # scale_paths_to_fit centers at (target_width/2, target_height/2) = (300, 450)
            # But drawing area starts at (50, 50), so center should be at (350, 500)
            # Offset = drawing_area_start + (target_width/2) - (target_width/2) = drawing_area_start
            # Actually simpler: scale_paths_to_fit centers at (target_width/2, target_height/2)
            # We need to add the drawing area's starting offset
            offset_x = drawing_area_x_limits[0]
            offset_y = drawing_area_y_limits[0]

            # Apply offset to all paths and ensure they're within bounds
            adjusted_paths = []
            for path in scaled_paths:
                adjusted_path = []
                for p in path:
                    new_x = p[0] + offset_x
                    new_y = p[1] + offset_y
                    # Constrain to drawing area bounds (safety check)
                    new_x = max(
                        drawing_area_x_limits[0], min(drawing_area_x_limits[1], new_x)
                    )
                    new_y = max(
                        drawing_area_y_limits[0], min(drawing_area_y_limits[1], new_y)
                    )
                    adjusted_path.append((new_x, new_y))
                adjusted_paths.append(adjusted_path)
            scaled_paths = adjusted_paths
        else:
            # Use paths as-is, but constrain to drawing area bounds (clip if out of bounds)
            scaled_paths = []
            for path in paths:
                constrained_path = []
                for p in path:
                    new_x = max(
                        drawing_area_x_limits[0], min(drawing_area_x_limits[1], p[0])
                    )
                    new_y = max(
                        drawing_area_y_limits[0], min(drawing_area_y_limits[1], p[1])
                    )
                    constrained_path.append((new_x, new_y))
                scaled_paths.append(constrained_path)

        # Send to robot if requested
        if send_to_robot and self.client:
            try:
                result = self.client.send_paths(scaled_paths, scale_to_fit=False)
                print(f"Sent to robot: {result}")
            except Exception as e:
                print(f"Error sending to robot: {e}")

        # Add to queue for animation
        for path in scaled_paths:
            if len(path) > 0:
                # First point: move with pen up
                self.path_queue.put((path[0][0], path[0][1], False))

                # Remaining points: draw with pen down
                for point in path[1:]:
                    self.path_queue.put((point[0], point[1], True))

                # Lift pen after path
                if len(path) > 1:
                    self.path_queue.put((path[-1][0], path[-1][1], False))

    def load_paths_from_json(self, json_file: str):
        """Load paths from a JSON file."""
        with open(json_file, "r") as f:
            data = json.load(f)

        if isinstance(data, dict) and "paths" in data:
            paths = data["paths"]
        elif isinstance(data, list):
            paths = data
        else:
            raise ValueError("Invalid JSON format")

        # Convert to list of tuples
        result = []
        for path in paths:
            point_list = [(float(p[0]), float(p[1])) for p in path]
            result.append(point_list)

        return result

    def animate(self, frame):
        """Animation update function."""
        # Check if enough time has passed since last update (for delay)
        current_time = time.time()
        if current_time - self.last_update_time < self.animation_delay:
            # Not enough time has passed, skip this frame
            return (
                self.gondola_black,
                self.gondola_red,
                self.gondola_left_wing,
                self.gondola_right_wing,
                self.left_belt_line,
                self.right_belt_line,
                self.current_drawing_line,
                self.transit_line,
                self.status_text,
            )

        if not self.path_queue.empty():
            x, y, pen_down = self.path_queue.get()
            self.last_update_time = current_time

            # Update position
            old_x, old_y = self.current_x, self.current_y
            self.current_x, self.current_y = x, y
            self.pen_down = pen_down

            # Update gondola
            self.update_gondola(x, y)

            # Update drawing paths
            if pen_down:
                # Drawing with pen down
                current_data = self.current_drawing_line.get_data()
                if len(current_data[0]) == 0:
                    # Start new drawing line
                    self.current_drawing_line.set_data([old_x, x], [old_y, y])
                else:
                    # Extend current drawing line
                    xs = list(current_data[0]) + [x]
                    ys = list(current_data[1]) + [y]
                    self.current_drawing_line.set_data(xs, ys)
            else:
                # Pen up - transit
                if len(self.current_drawing_line.get_data()[0]) > 0:
                    # Save current drawing line and start new one
                    line_data = self.current_drawing_line.get_data()
                    (new_line,) = self.ax.plot(
                        line_data[0], line_data[1], "b-", linewidth=2, zorder=20
                    )
                    self.drawing_lines.append(new_line)
                    self.current_drawing_line.set_data([], [])

                # Add to transit path
                transit_data = self.transit_line.get_data()
                if len(transit_data[0]) == 0:
                    self.transit_line.set_data([old_x, x], [old_y, y])
                else:
                    xs = list(transit_data[0]) + [old_x, x]
                    ys = list(transit_data[1]) + [old_y, y]
                    self.transit_line.set_data(xs, ys)

            # Update status
            Z1, Z2 = self.calculate_belt_lengths(x, y)
            status = f"Position: ({x:.1f}, {y:.1f}) mm\n"
            status += f"Pen: {'DOWN' if pen_down else 'UP'}\n"
            status += f"Belt L1: {Z1:.1f} mm\n"
            status += f"Belt L2: {Z2:.1f} mm\n"
            if self.robot_ip:
                status += f"Robot: {self.robot_ip}\n"
            status += f"Queue: {self.path_queue.qsize()} points"
            self.status_text.set_text(status)

        return (
            self.gondola_black,
            self.gondola_red,
            self.gondola_left_wing,
            self.gondola_right_wing,
            self.left_belt_line,
            self.right_belt_line,
            self.current_drawing_line,
            self.transit_line,
            self.status_text,
        )

    def start_animation(self):
        """Start the animation."""
        if not self.is_animating:
            self.ani = animation.FuncAnimation(
                self.fig, self.animate, interval=50, blit=False, cache_frame_data=False
            )
            self.is_animating = True
            plt.show()

    def monitor_robot(self, interval: float = 1.0):
        """
        Monitor robot status and visualize (if robot is connected).

        Args:
            interval: Check interval in seconds
        """
        if not self.client:
            print("No robot connection configured")
            return

        def monitor_thread():
            while True:
                try:
                    status = self.client.get_status()
                    print(f"Robot status: {status}")
                except Exception as e:
                    print(f"Error monitoring robot: {e}")
                time.sleep(interval)

        threading.Thread(target=monitor_thread, daemon=True).start()


def main():
    parser = argparse.ArgumentParser(
        description="Drawing Robot Simulation UI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Simulate from JSON file
  python drawing_simulator.py --file vectors.json
  
  # Connect to robot and send paths
  python drawing_simulator.py --file vectors.json --robot 192.168.4.1 --send
  
  # Simulate from image
  python drawing_simulator.py --image logo.png
        """,
    )

    parser.add_argument("--file", "-f", help="JSON file with vector paths")
    parser.add_argument("--image", "-i", help="Image file to convert and simulate")
    parser.add_argument(
        "--robot", "-r", help="Robot IP address (for monitoring/sending)"
    )
    parser.add_argument("--send", action="store_true", help="Send paths to robot")
    parser.add_argument("--no-scale", action="store_true", help="Do not scale paths")
    parser.add_argument(
        "--delay",
        "-d",
        type=float,
        default=0.1,
        help="Animation delay between points in seconds (default: 0.1s = 100ms). "
        "Increase for slower animation (e.g., 0.5 for 500ms delay)",
    )

    args = parser.parse_args()

    # Create simulator with animation delay
    simulator = DrawingSimulator(robot_ip=args.robot, animation_delay=args.delay)

    # Load paths
    paths = []
    if args.file:
        print(f"Loading paths from {args.file}...")
        paths = simulator.load_paths_from_json(args.file)
    elif args.image:
        print(f"Converting image {args.image} to vectors...")
        from image_to_vectors import ImageToVectorConverter

        converter = ImageToVectorConverter()
        paths = converter.image_to_vectors(args.image)
    else:
        print("No input file specified. Use --file or --image")
        print(
            "Starting simulator with empty paths (you can add paths programmatically)"
        )

    if paths:
        print(f"Loaded {len(paths)} path(s)")
        total_points = sum(len(path) for path in paths)
        print(f"Total points: {total_points}")

        # Add paths to simulator
        scale_to_fit = not args.no_scale
        if args.no_scale:
            print(
                "Scaling disabled - using coordinates as-is (may be clipped if out of bounds)"
            )
        else:
            print("Scaling enabled - paths will be scaled to fit drawing area")
        simulator.add_path(paths, send_to_robot=args.send, scale_to_fit=scale_to_fit)

    # Start monitoring if robot connected
    if args.robot:
        simulator.monitor_robot()

    # Start animation
    print("Starting simulation...")
    simulator.start_animation()


if __name__ == "__main__":
    main()
