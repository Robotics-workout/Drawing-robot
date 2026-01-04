"""
Send waypoints directly to the drawing robot without image processing.
Useful for sending custom coordinates or pre-computed paths.
"""

import requests
import json
import argparse
from typing import List, Tuple
from image_to_vectors import DrawingRobotClient, scale_paths_to_fit


def parse_waypoints_file(file_path: str) -> List[List[Tuple[float, float]]]:
    """
    Parse waypoints from a JSON file.
    
    Expected format:
    {
        "paths": [
            [[x1, y1], [x2, y2], ...],
            [[x3, y3], [x4, y4], ...]
        ]
    }
    Or simple format:
    [
        [[x1, y1], [x2, y2], ...],
        [[x3, y3], [x4, y4], ...]
    ]
    """
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    if isinstance(data, dict) and 'paths' in data:
        paths = data['paths']
    elif isinstance(data, list):
        paths = data
    else:
        raise ValueError("Invalid JSON format. Expected 'paths' key or array of paths.")
    
    # Convert to list of tuples
    result = []
    for path in paths:
        if not isinstance(path, list):
            raise ValueError("Each path must be a list of coordinates")
        point_list = []
        for point in path:
            if len(point) < 2:
                raise ValueError("Each point must have at least x and y coordinates")
            point_list.append((float(point[0]), float(point[1])))
        result.append(point_list)
    
    return result


def parse_waypoints_string(waypoints_str: str) -> List[List[Tuple[float, float]]]:
    """
    Parse waypoints from a string.
    
    Format: "x1,y1 x2,y2 x3,y3 | x4,y4 x5,y5"
    (pipe separates paths, space separates points, comma separates x,y)
    """
    paths = []
    path_strings = waypoints_str.split('|')
    
    for path_str in path_strings:
        path_str = path_str.strip()
        if not path_str:
            continue
        
        points = []
        point_strings = path_str.split()
        for point_str in point_strings:
            coords = point_str.split(',')
            if len(coords) < 2:
                raise ValueError(f"Invalid point format: {point_str}")
            x = float(coords[0])
            y = float(coords[1])
            points.append((x, y))
        
        if points:
            paths.append(points)
    
    return paths


def main():
    parser = argparse.ArgumentParser(
        description='Send waypoints directly to the drawing robot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # From JSON file
  python send_waypoints.py waypoints.json
  
  # From command line string
  python send_waypoints.py --waypoints "100,100 200,100 200,200 100,200 100,100"
  
  # Multiple paths (pipe-separated)
  python send_waypoints.py --waypoints "100,100 200,100 200,200 100,200 | 300,300 400,300 400,400"
  
  # From file without scaling
  python send_waypoints.py waypoints.json --no-scale
        """
    )
    
    parser.add_argument('file', nargs='?', help='JSON file containing waypoints')
    parser.add_argument('--waypoints', '-w', help='Waypoints as string: "x1,y1 x2,y2 | x3,y3 x4,y4"')
    parser.add_argument('--ip', default='192.168.4.1', help='Robot IP address (default: 192.168.4.1)')
    parser.add_argument('--no-scale', action='store_true', 
                       help='Do not scale waypoints to fit drawing area (use coordinates as-is)')
    parser.add_argument('--save-json', help='Save waypoints to JSON file before sending')
    
    args = parser.parse_args()
    
    # Get waypoints from file or string
    if args.file:
        print(f"Loading waypoints from {args.file}...")
        paths = parse_waypoints_file(args.file)
    elif args.waypoints:
        print("Parsing waypoints from string...")
        paths = parse_waypoints_string(args.waypoints)
    else:
        parser.error("Either provide a file or use --waypoints option")
    
    print(f"Loaded {len(paths)} path(s)")
    total_points = sum(len(path) for path in paths)
    print(f"Total waypoints: {total_points}")
    
    # Display waypoints
    for i, path in enumerate(paths):
        print(f"  Path {i+1}: {len(path)} points")
        if len(path) <= 5:
            for point in path:
                print(f"    ({point[0]:.2f}, {point[1]:.2f})")
        else:
            print(f"    First: ({path[0][0]:.2f}, {path[0][1]:.2f})")
            print(f"    Last: ({path[-1][0]:.2f}, {path[-1][1]:.2f})")
    
    # Save to JSON if requested
    if args.save_json:
        json_paths = [[[float(p[0]), float(p[1])] for p in path] for path in paths]
        with open(args.save_json, 'w') as f:
            json.dump({"paths": json_paths}, f, indent=2)
        print(f"\nSaved waypoints to {args.save_json}")
    
    # Connect to robot and send
    print(f"\nConnecting to robot at {args.ip}...")
    client = DrawingRobotClient(args.ip)
    
    # Check status
    status = client.get_status()
    print(f"Robot status: {status}")
    
    # Send waypoints
    scale_to_fit = not args.no_scale
    if scale_to_fit:
        print("Sending waypoints (will be scaled to fit drawing area)...")
    else:
        print("Sending waypoints (using coordinates as-is, may be clipped if out of bounds)...")
    
    result = client.send_paths(paths, scale_to_fit=scale_to_fit)
    print(f"Response: {result}")


if __name__ == "__main__":
    main()

