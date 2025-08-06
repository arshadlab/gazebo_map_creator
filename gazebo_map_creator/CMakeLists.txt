cmake_minimum_required(VERSION 3.10)
project(gazebo_map_creator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override)
endif()

# Platform defines
if(WIN32)
  add_compile_definitions(
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_map_creator_interface REQUIRED)  # your custom service package
find_package(PCL REQUIRED)

# Gazebo Sim dependencies
find_package(gz-sim8 REQUIRED) # adjust if you're using a newer sim version
find_package(gz-plugin2 REQUIRED)

find_package(PCL REQUIRED)

find_package(gz-msgs10 REQUIRED)


include_directories(
  ${PCL_INCLUDE_DIRS}
)

include_directories(
  ${gz-msgs10_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)



add_definitions(${PCL_DEFINITIONS})

# Plugin library
add_library(map_creator_plugin SHARED
  src/gazbeo_map_creator.cpp
)

ament_target_dependencies(map_creator_plugin
  rclcpp
  gazebo_map_creator_interface
  gz-sim8
  gz-msgs10
  gz-plugin2
)

target_link_libraries(map_creator_plugin
  gz-msgs10::gz-msgs10
  ${PCL_LIBRARIES}
)

# Export and install
install(TARGETS map_creator_plugin
  LIBRARY DESTINATION lib
)

# Optional: Python script for ROS request (if you have one)
install(PROGRAMS
  scripts/request_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
