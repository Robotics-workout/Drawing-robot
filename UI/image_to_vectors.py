"""
Client script to convert images to vectors and send to the drawing robot via WiFi.
This script performs edge detection and converts the result to vector paths.
"""

import requests
import numpy as np
from PIL import Image
import cv2
import json
from typing import List, Tuple


class ImageToVectorConverter:
    """Converts images to vector paths suitable for the drawing robot."""

    def __init__(
        self, drawing_area_width: float = 600.0, drawing_area_height: float = 950.0
    ):
        """
        Initialize the converter.

        Args:
            drawing_area_width: Width of drawing area in mm (BASE - 2*BIAS = 700 - 100 = 600)
            drawing_area_height: Height of drawing area in mm (1000 - 100 = 900, using 950 for safety)
        """
        self.drawing_area_width = drawing_area_width
        self.drawing_area_height = drawing_area_height

    def load_image(self, image_path: str) -> np.ndarray:
        """Load an image from file."""
        img = Image.open(image_path)
        return np.array(img)

    def preprocess_image(
        self, image: np.ndarray, target_size: Tuple[int, int] = (400, 600)
    ) -> np.ndarray:
        """
        Preprocess image for edge detection.

        Args:
            image: Input image as numpy array
            target_size: Target size (width, height) for processing

        Returns:
            Grayscale, resized image
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        else:
            gray = image

        # Resize to target size (smaller for faster processing)
        gray = cv2.resize(gray, target_size, interpolation=cv2.INTER_AREA)

        return gray

    def detect_edges(
        self,
        image: np.ndarray,
        low_threshold: int = 50,
        high_threshold: int = 150,
        method: str = "canny",
    ) -> np.ndarray:
        """
        Detect edges in the image.

        Args:
            image: Grayscale image
            low_threshold: Lower threshold for Canny edge detection
            high_threshold: Upper threshold for Canny edge detection
            method: Edge detection method ('canny' or 'threshold')

        Returns:
            Binary edge image
        """
        if method == "canny":
            edges = cv2.Canny(image, low_threshold, high_threshold)
        elif method == "threshold":
            # Adaptive threshold for better results on varying lighting
            edges = cv2.adaptiveThreshold(
                image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
            )
        else:
            raise ValueError(f"Unknown method: {method}")

        return edges

    def trace_contours(
        self,
        edge_image: np.ndarray,
        min_contour_length: int = 10,
        simplify_epsilon: float = 1.0,
    ) -> List[List[Tuple[float, float]]]:
        """
        Trace contours from edge image and convert to paths.

        Args:
            edge_image: Binary edge image
            min_contour_length: Minimum contour length to include
            simplify_epsilon: Epsilon for contour simplification (Douglas-Peucker)

        Returns:
            List of paths, where each path is a list of (x, y) coordinates
        """
        # Find contours
        contours, _ = cv2.findContours(
            edge_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        paths = []
        height, width = edge_image.shape

        for contour in contours:
            if len(contour) < min_contour_length:
                continue

            # Simplify contour using Douglas-Peucker algorithm
            epsilon = simplify_epsilon
            simplified = cv2.approxPolyDP(contour, epsilon, closed=False)

            # Convert to list of points and scale to drawing area
            path = []
            for point in simplified:
                x = point[0][0]
                y = point[0][1]

                # Scale coordinates to drawing area (0-width -> 0-drawing_area_width)
                scaled_x = (x / width) * self.drawing_area_width
                scaled_y = (y / height) * self.drawing_area_height

                # Add offset to center in drawing area (assuming X_BIAS = 50, Y_BIAS = 50)
                scaled_x += 50  # X_BIAS
                scaled_y += 50  # Y_BIAS

                path.append((scaled_x, scaled_y))

            if len(path) > 1:
                paths.append(path)

        return paths

    def image_to_vectors(
        self,
        image_path: str,
        target_size: Tuple[int, int] = (400, 600),
        edge_method: str = "canny",
        low_threshold: int = 50,
        high_threshold: int = 150,
        min_contour_length: int = 10,
        simplify_epsilon: float = 1.0,
    ) -> List[List[Tuple[float, float]]]:
        """
        Complete pipeline: load image, detect edges, and convert to vectors.

        Args:
            image_path: Path to input image
            target_size: Target size for processing (width, height)
            edge_method: Edge detection method ('canny' or 'threshold')
            low_threshold: Lower threshold for Canny
            high_threshold: Upper threshold for Canny
            min_contour_length: Minimum contour length
            simplify_epsilon: Contour simplification epsilon

        Returns:
            List of vector paths
        """
        # Load and preprocess
        image = self.load_image(image_path)
        processed = self.preprocess_image(image, target_size)

        # Detect edges
        edges = self.detect_edges(processed, low_threshold, high_threshold, edge_method)

        # Trace contours
        paths = self.trace_contours(edges, min_contour_length, simplify_epsilon)

        return paths


def scale_paths_to_fit(
    paths: List[List[Tuple[float, float]]],
    target_width: float,
    target_height: float,
    margin: float = 10.0,
) -> List[List[Tuple[float, float]]]:
    """
    Scale paths to fit within target dimensions while maintaining aspect ratio.

    Args:
        paths: List of paths, where each path is a list of (x, y) coordinates
        target_width: Target width in mm
        target_height: Target height in mm
        margin: Margin to leave around edges in mm

    Returns:
        Scaled paths that fit within the target area
    """
    if not paths or not any(paths):
        return paths

    # Find bounding box of all points
    all_points = [point for path in paths for point in path]
    if not all_points:
        return paths

    min_x = min(p[0] for p in all_points)
    max_x = max(p[0] for p in all_points)
    min_y = min(p[1] for p in all_points)
    max_y = max(p[1] for p in all_points)

    # Calculate source dimensions
    src_width = max_x - min_x
    src_height = max_y - min_y

    if src_width == 0 or src_height == 0:
        # All points are the same or on a line, center them
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        offset_x = (target_width / 2) - center_x
        offset_y = (target_height / 2) - center_y

        scaled_paths = []
        for path in paths:
            scaled_path = [(p[0] + offset_x, p[1] + offset_y) for p in path]
            scaled_paths.append(scaled_path)
        return scaled_paths

    # Calculate available area (with margins)
    available_width = target_width - (2 * margin)
    available_height = target_height - (2 * margin)

    # Calculate scale factors to fit both dimensions
    scale_x = available_width / src_width
    scale_y = available_height / src_height
    scale = min(scale_x, scale_y)  # Use smaller scale to maintain aspect ratio

    # Calculate center offset
    scaled_width = src_width * scale
    scaled_height = src_height * scale
    offset_x = (target_width / 2) - ((min_x + max_x) / 2 * scale)
    offset_y = (target_height / 2) - ((min_y + max_y) / 2 * scale)

    # Scale and translate all paths
    scaled_paths = []
    for path in paths:
        scaled_path = [(p[0] * scale + offset_x, p[1] * scale + offset_y) for p in path]
        scaled_paths.append(scaled_path)

    return scaled_paths


class DrawingRobotClient:
    """Client to send vector paths to the drawing robot via WiFi."""

    def __init__(
        self,
        robot_ip: str = "192.168.4.1",
        port: int = 80,
        drawing_area_width: float = 600.0,
        drawing_area_height: float = 950.0,
    ):
        """
        Initialize the client.

        Args:
            robot_ip: IP address of the robot (default AP IP)
            port: Port number (default 80)
            drawing_area_width: Width of drawing area in mm
            drawing_area_height: Height of drawing area in mm
        """
        self.base_url = f"http://{robot_ip}:{port}"
        self.drawing_area_width = drawing_area_width
        self.drawing_area_height = drawing_area_height

    def send_paths(
        self, paths: List[List[Tuple[float, float]]], scale_to_fit: bool = True
    ) -> dict:
        """
        Send vector paths to the robot.

        Args:
            paths: List of paths, where each path is a list of (x, y) coordinates
            scale_to_fit: If True, scale paths to fit within drawing area

        Returns:
            Response from the server
        """
        # Scale paths to fit if requested
        if scale_to_fit:
            paths = scale_paths_to_fit(
                paths, self.drawing_area_width, self.drawing_area_height
            )

        # Convert paths to JSON format
        json_paths = [[[float(p[0]), float(p[1])] for p in path] for path in paths]
        data = {"paths": json_paths}

        # Send POST request
        url = f"{self.base_url}/draw"
        try:
            response = requests.post(url, json=data, timeout=10)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error sending paths: {e}")
            return {"status": "error", "message": str(e)}

    def get_status(self) -> dict:
        """Get the status of the robot."""
        url = f"{self.base_url}/status"
        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error getting status: {e}")
            return {"status": "error", "message": str(e)}


def main():
    """Example usage of the image to vector converter and client."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert image to vectors and send to drawing robot"
    )
    parser.add_argument("image_path", help="Path to input image")
    parser.add_argument(
        "--ip", default="192.168.4.1", help="Robot IP address (default: 192.168.4.1)"
    )
    parser.add_argument(
        "--method",
        choices=["canny", "threshold"],
        default="canny",
        help="Edge detection method (default: canny)",
    )
    parser.add_argument(
        "--low-threshold",
        type=int,
        default=50,
        help="Lower threshold for Canny edge detection (default: 50)",
    )
    parser.add_argument(
        "--high-threshold",
        type=int,
        default=150,
        help="Upper threshold for Canny edge detection (default: 150)",
    )
    parser.add_argument(
        "--min-length",
        type=int,
        default=10,
        help="Minimum contour length (default: 10)",
    )
    parser.add_argument(
        "--epsilon",
        type=float,
        default=1.0,
        help="Contour simplification epsilon (default: 1.0)",
    )
    parser.add_argument(
        "--no-send",
        action="store_true",
        help="Only convert to vectors, do not send to robot",
    )
    parser.add_argument("--save-json", help="Save vectors to JSON file")

    args = parser.parse_args()

    # Convert image to vectors
    print(f"Loading image: {args.image_path}")
    converter = ImageToVectorConverter()

    print("Converting image to vectors...")
    paths = converter.image_to_vectors(
        args.image_path,
        edge_method=args.method,
        low_threshold=args.low_threshold,
        high_threshold=args.high_threshold,
        min_contour_length=args.min_length,
        simplify_epsilon=args.epsilon,
    )

    print(f"Found {len(paths)} paths")
    total_points = sum(len(path) for path in paths)
    print(f"Total points: {total_points}")

    # Save to JSON if requested
    if args.save_json:
        json_paths = [[[float(p[0]), float(p[1])] for p in path] for path in paths]
        with open(args.save_json, "w") as f:
            json.dump({"paths": json_paths}, f, indent=2)
        print(f"Saved vectors to {args.save_json}")

    # Send to robot if requested
    if not args.no_send:
        print(f"\nConnecting to robot at {args.ip}...")
        client = DrawingRobotClient(args.ip)

        # Check status first
        status = client.get_status()
        print(f"Robot status: {status}")

        # Send paths (with scaling to fit)
        print("Sending paths to robot (scaled to fit drawing area)...")
        result = client.send_paths(paths, scale_to_fit=True)
        print(f"Response: {result}")
    else:
        print("\nSkipping send (--no-send flag set)")


if __name__ == "__main__":
    main()
