cmake_minimum_required(VERSION 3.5)
project(gazebo_map_creator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_map_creator_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(gazebo_map_creator SHARED
  src/gazbeo_map_creator.cpp
)

ament_target_dependencies(gazebo_map_creator
  "gazebo_map_creator_interface"
  "geometry_msgs"
  "gazebo_dev"
  "gazebo_ros"
)

ament_export_libraries(gazebo_map_creator)
ament_export_dependencies(gazebo_dev)

target_link_libraries(gazebo_map_creator
  ${GAZEBO_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(PROGRAMS DESTINATION lib/${PROJECT_NAME}/ )
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(TARGETS gazebo_map_creator LIBRARY DESTINATION lib )
# Install Python executables
install(PROGRAMS
  scripts/request_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
