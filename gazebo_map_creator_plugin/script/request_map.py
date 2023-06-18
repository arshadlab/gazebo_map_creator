#!/usr/bin/env python3
# Copyright (c) 2023 arshadlab.
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
import rclpy
import argparse
from rclpy.node import Node
from gazebo_map_creator_interface.srv import MapRequest
from geometry_msgs.msg import Point

def get_corners(lower_right, upper_left):
    lower_left = (lower_right[0], upper_left[1])
    upper_right = (upper_left[0], lower_right[1])
    return upper_left, upper_right, lower_right, lower_left

def get_corners(point_string):
    point_string = point_string.strip("()")  # Remove outer parentheses if present
    points = point_string.split(")(")  # Split string into individual points

    if len(points) != 2:
        raise ValueError("Invalid input format. Expected two points.")

    x1, y1 = map(float, points[0].split(","))
    x2, y2 = map(float, points[1].split(","))

    lower_right = Point(x=x1, y=y1)
    upper_left = Point(x=x2, y=y2)
    lower_left = Point(x=x1, y=y2)
    upper_right = Point(x=x2, y=y1)    
    

    return upper_left, upper_right, lower_right, lower_left

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--corners", help="Upper left and lower right corner (lower_right, upperleft) x-> up, y -> left", default="(-10,-10)(10,10)", type=str)
    parser.add_argument("-H", "--height", help="Maximum height to consider", default=1.0, type=float)
    parser.add_argument("-g", "--ground_height", help="Ground base height", default=0.001, type=float)
    parser.add_argument("-t", "--threshold", help="Threshold to use", default=255, type=int)
    parser.add_argument("-r", "--resolution", help="Resolution", default=0.01, type=float)
    parser.add_argument("-d", "--distance", help="Resolution", default=0.02, type=float)
    parser.add_argument("-f", "--filename", help="Output file base name.  e.g basename.png & basename.pgm", default="map", type=str)

    

    arg_list = parser.parse_args()

    request = MapRequest.Request()
    upperleft, upperright, lowerright, lowerleft = get_corners(arg_list.corners)
    request.upperleft = upperleft
    request.upperright = upperright
    request.lowerright = lowerright
    request.lowerleft = lowerleft

    request.height = arg_list.height
    request.ground_height=arg_list.ground_height
    request.threshold = arg_list.threshold
    request.resolution = arg_list.resolution
    request.distance = arg_list.distance
    request.filename = arg_list.filename
        
        

    node = Node("map_request")

    print("Request:",
            " UL.x:", request.upperleft.x,
            " UL.y:", request.upperleft.y,
            " UR.x:", request.upperright.x,
            " UR.y:", request.upperright.y,
            " LR.x:", request.lowerright.x,
            " LR.y:", request.lowerright.y,
            " LL.x:", request.lowerleft.x,
            " LL.y:", request.lowerleft.y,
            " Height:", request.height,
            " Ground Height:", request.ground_height,
            " Resolution:", request.resolution,
            " Distance:", request.distance,
            " Filename:", request.filename,
            " Threshold:", request.threshold)

    service_name = '/world/save_map'
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
