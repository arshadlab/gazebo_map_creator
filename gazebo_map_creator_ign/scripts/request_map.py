#!/usr/bin/env python3
# Copyright (c) 2023 .
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import re
import rclpy
import argparse
from rclpy.node import Node
from gazebo_map_creator_interface.srv import MapRequest
from geometry_msgs.msg import Point

def parse_coordinates(param):
    # Use regex pattern to extract XYZ values from the parameter
    pattern = r'\(\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*,\s*(-?\d+\.\d+)\s*\)'
    matches = re.findall(pattern, param)

    if len(matches) != 2:
        print("Error: Invalid number of coordinate pairs")
        return None, None

    # Convert the extracted values to float and create Point variables
    point1 = Point(x = float(matches[0][0]), y = float(matches[0][1]), z = float(matches[0][2]))
    point2 = Point(x = float(matches[1][0]), y = float(matches[1][1]), z = float(matches[1][2]))

    return point1, point2

def main(args=None):
    rclpy.init(args=args)
    request = MapRequest.Request()

    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-c", "--corners", help="lower right and upper left corner (lower_right, upperleft) x facing up, y facing left", default="(-10.0,-10.0, 0.05)(10.0,10.0, 10.0)", type=str)
    parser.add_argument("-r", "--resolution", help="Output map resolution: lower the value, higher the output map resolution", default=request.resolution, type=float)
    parser.add_argument("-d", "--multiplier", help="Collision distance multiplier to be used during map creation. e.g half of step size = 0.5", default=request.range_multiplier, type=float)
    parser.add_argument("--skip-vertical-scan", help="Skip full scan resulting in faster results. Recommended for 2D map focused results", action="store_true")
    parser.add_argument("-t", "--threshold", help="Pixel threshold value to use in 2D map", default=request.threshold_2d, type=int)
    parser.add_argument("-f", "--filename", help="Output file base name.  e.g basename.pcd, basename.pgm, basename.yaml & basename.png", default=request.filename, type=str)

    arg_list = parser.parse_args()

    request.lowerright, request.upperleft = parse_coordinates(arg_list.corners)
    service_name = '/world/save_map'

    request.resolution = arg_list.resolution
    request.range_multiplier = arg_list.multiplier
    request.filename = arg_list.filename
    request.threshold_2d = arg_list.threshold
    request.skip_vertical_scan = arg_list.skip_vertical_scan
    
    node = Node("map_request")

    print(f"Request:\n"
      f" Upper Left  : ({request.upperleft.x}, {request.upperleft.y}, {request.upperleft.z})\n"
      f" Lower Right : ({request.lowerright.x}, {request.lowerright.y}, {request.lowerright.z})\n"
      f" Resolution: {request.resolution}\n"
      f" Fast mode:  {arg_list.skip_vertical_scan}\n"
      f" Range Multiplier: {request.range_multiplier}\n"
      f" Threshold: {request.threshold_2d}\n"
      f" Filename: {request.filename}[.pcd, .pgm, .png, .yaml]")

    client = node.create_client(MapRequest, service_name)
    while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'service "{service_name}" not available, waiting again...')

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    print(f"Results are: {future.result()}")
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
