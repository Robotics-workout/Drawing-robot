"""
Examples and tutorials for using the Drawing Robot system.
This script demonstrates various ways to use the image_to_vectors and send_waypoints tools.
"""

import json
from image_to_vectors import ImageToVectorConverter, DrawingRobotClient, scale_paths_to_fit
from send_waypoints import parse_waypoints_string


def example_1_basic_image_conversion():
    """Example 1: Basic image to vector conversion."""
    print("=" * 60)
    print("Example 1: Basic Image to Vector Conversion")
    print("=" * 60)
    print("""
This example shows how to convert an image to vectors.

Command:
    python image_to_vectors.py your_image.png

What it does:
    1. Loads your image
    2. Converts to grayscale
    3. Detects edges (Canny edge detection)
    4. Traces contours to create paths
    5. Scales paths to fit drawing area
    6. Sends to robot via WiFi

Example:
    python image_to_vectors.py logo.png
    """)


def example_2_custom_edge_detection():
    """Example 2: Custom edge detection settings."""
    print("=" * 60)
    print("Example 2: Custom Edge Detection Settings")
    print("=" * 60)
    print("""
Adjust edge detection parameters for better results.

Commands:
    # Use threshold method instead of Canny
    python image_to_vectors.py image.png --method threshold
    
    # Adjust Canny thresholds (lower = more edges, higher = fewer edges)
    python image_to_vectors.py image.png --low-threshold 30 --high-threshold 100
    
    # More detailed paths (lower epsilon = more points)
    python image_to_vectors.py image.png --epsilon 0.5
    
    # Filter out small contours
    python image_to_vectors.py image.png --min-length 20

Tips:
    - Low threshold: Start with 30-50 for detailed images
    - High threshold: Start with 100-150 for clean images
    - Epsilon: Lower values (0.5-1.0) = more detail, higher (2.0-5.0) = simpler
    """)


def example_3_send_waypoints():
    """Example 3: Send waypoints directly."""
    print("=" * 60)
    print("Example 3: Send Waypoints Directly")
    print("=" * 60)
    print("""
Send coordinates directly without image processing.

Commands:
    # From command line (single path)
    python send_waypoints.py --waypoints "100,100 200,100 200,200 100,200 100,100"
    
    # Multiple paths (pipe-separated)
    python send_waypoints.py --waypoints "100,100 200,100 200,200 | 300,300 400,300 400,400"
    
    # From JSON file
    python send_waypoints.py waypoints.json
    
    # Without scaling (use coordinates as-is)
    python send_waypoints.py waypoints.json --no-scale

Format:
    - Points: "x,y" (comma-separated)
    - Paths: Space-separated points
    - Multiple paths: Pipe-separated "|"
    """)


def example_4_create_waypoints_json():
    """Example 4: Create waypoints JSON file."""
    print("=" * 60)
    print("Example 4: Create Waypoints JSON File")
    print("=" * 60)
    
    # Example waypoints data
    waypoints = {
        "paths": [
            # Square
            [[100, 100], [200, 100], [200, 200], [100, 200], [100, 100]],
            # Triangle
            [[300, 300], [400, 300], [350, 400], [300, 300]],
            # Line
            [[500, 100], [600, 500]]
        ]
    }
    
    print("""
Create a JSON file with waypoints:

Example waypoints.json:
""")
    print(json.dumps(waypoints, indent=2))
    
    print("""
Save this to a file (e.g., waypoints.json) and use:
    python send_waypoints.py waypoints.json
    """)


def example_5_programmatic_usage():
    """Example 5: Use the libraries programmatically."""
    print("=" * 60)
    print("Example 5: Programmatic Usage")
    print("=" * 60)
    print("""
Use the libraries in your own Python scripts:

Example code:
""")
    
    code_example = '''
from image_to_vectors import ImageToVectorConverter, DrawingRobotClient

# Convert image to vectors
converter = ImageToVectorConverter()
paths = converter.image_to_vectors("image.png", 
                                    edge_method="canny",
                                    low_threshold=50,
                                    high_threshold=150)

# Send to robot
client = DrawingRobotClient("192.168.4.1")
result = client.send_paths(paths, scale_to_fit=True)
print(result)
'''
    print(code_example)


def example_6_workflow():
    """Example 6: Complete workflow."""
    print("=" * 60)
    print("Example 6: Complete Workflow")
    print("=" * 60)
    print("""
Complete workflow from image to drawing:

Step 1: Convert image and save vectors
    python image_to_vectors.py logo.png --save-json vectors.json --no-send

Step 2: Review/edit vectors.json if needed

Step 3: Send to robot
    python send_waypoints.py vectors.json

Or do it all at once:
    python image_to_vectors.py logo.png
    """)


def example_7_troubleshooting():
    """Example 7: Troubleshooting tips."""
    print("=" * 60)
    print("Example 7: Troubleshooting Tips")
    print("=" * 60)
    print("""
Common issues and solutions:

1. "Connection refused" or "Connection timeout"
   - Check robot is powered on
   - Verify IP address (check Serial Monitor)
   - Ensure you're on the same WiFi network
   - Try: python send_waypoints.py --ip 192.168.1.100

2. "Too many edges" or "Drawing too detailed"
   - Increase epsilon: --epsilon 2.0
   - Increase min-length: --min-length 20
   - Use threshold method: --method threshold

3. "Not enough edges" or "Missing details"
   - Decrease thresholds: --low-threshold 30 --high-threshold 100
   - Decrease epsilon: --epsilon 0.5
   - Use Canny method: --method canny

4. "Out of bounds" errors
   - Use automatic scaling (default)
   - Or manually scale coordinates in JSON file

5. "Robot not moving"
   - Check Serial Monitor for errors
   - Verify stepper motor connections
   - Check power supply
    """)


def example_8_advanced_waypoints():
    """Example 8: Advanced waypoint examples."""
    print("=" * 60)
    print("Example 8: Advanced Waypoint Examples")
    print("=" * 60)
    
    examples = {
        "Square": "100,100 200,100 200,200 100,200 100,100",
        "Triangle": "300,300 400,300 350,400 300,300",
        "Circle (approximation)": "350,200 380,220 400,250 380,280 350,300 320,280 300,250 320,220 350,200",
        "Letter A": "100,500 150,400 200,500 | 125,450 175,450",
        "Multiple shapes": "100,100 200,100 200,200 100,200 100,100 | 300,300 400,300 350,400 300,300"
    }
    
    print("\nExample waypoint strings:\n")
    for name, waypoints in examples.items():
        print(f"{name}:")
        print(f'  python send_waypoints.py --waypoints "{waypoints}"')
        print()


def interactive_demo():
    """Interactive demo mode."""
    print("=" * 60)
    print("Interactive Demo - Waypoint Creation")
    print("=" * 60)
    print("\nLet's create a simple square waypoint:")
    
    # Create a simple square
    square_path = [[100, 100], [200, 100], [200, 200], [100, 200], [100, 100]]
    paths = [square_path]
    
    print("\nCreated square path:")
    for point in square_path:
        print(f"  ({point[0]}, {point[1]})")
    
    print("\nAs JSON:")
    json_data = {"paths": paths}
    print(json.dumps(json_data, indent=2))
    
    print("\nAs command line string:")
    waypoint_str = " ".join([f"{p[0]},{p[1]}" for p in square_path])
    print(f'  python send_waypoints.py --waypoints "{waypoint_str}"')
    
    print("\nTo send this to robot:")
    print(f'  python send_waypoints.py --waypoints "{waypoint_str}"')


def main():
    """Main menu for examples."""
    examples = {
        "1": ("Basic Image Conversion", example_1_basic_image_conversion),
        "2": ("Custom Edge Detection", example_2_custom_edge_detection),
        "3": ("Send Waypoints", example_3_send_waypoints),
        "4": ("Create Waypoints JSON", example_4_create_waypoints_json),
        "5": ("Programmatic Usage", example_5_programmatic_usage),
        "6": ("Complete Workflow", example_6_workflow),
        "7": ("Troubleshooting", example_7_troubleshooting),
        "8": ("Advanced Waypoints", example_8_advanced_waypoints),
        "9": ("Interactive Demo", interactive_demo),
        "0": ("Show All Examples", None)
    }
    
    print("\n" + "=" * 60)
    print("Drawing Robot - Usage Examples and Tutorials")
    print("=" * 60)
    print("\nAvailable examples:")
    for key, (name, _) in examples.items():
        if key != "0":
            print(f"  {key}. {name}")
    print("  0. Show All Examples")
    print("  q. Quit")
    
    while True:
        choice = input("\nSelect an example (or 'q' to quit): ").strip().lower()
        
        if choice == "q":
            print("Goodbye!")
            break
        elif choice == "0":
            # Show all examples
            for key, (name, func) in examples.items():
                if key != "0" and func:
                    func()
                    input("\nPress Enter to continue...")
        elif choice in examples and examples[choice][1]:
            examples[choice][1]()
            input("\nPress Enter to continue...")
        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # Run specific example from command line
        example_num = sys.argv[1]
        examples = {
            "1": example_1_basic_image_conversion,
            "2": example_2_custom_edge_detection,
            "3": example_3_send_waypoints,
            "4": example_4_create_waypoints_json,
            "5": example_5_programmatic_usage,
            "6": example_6_workflow,
            "7": example_7_troubleshooting,
            "8": example_8_advanced_waypoints,
            "9": interactive_demo
        }
        if example_num in examples:
            examples[example_num]()
        else:
            print(f"Example {example_num} not found. Use 1-9.")
    else:
        # Interactive mode
        main()


