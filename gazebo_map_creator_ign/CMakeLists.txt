cmake_minimum_required(VERSION 3.5)
project(gazebo_map_creator_ign)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCL_LIBRARIES)
set(GZ_PLUGIN)
set(GZ_SIM)
set(GZ_RENDERING)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
#find_package(PCL 1.2 REQUIRED)
find_package(gazebo_map_creator_interface REQUIRED)




if("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  find_package(gz-plugin2 REQUIRED)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
  set(GZ_PLUGIN gz-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM gz-sim${GZ_SIM_VER}::core)
  add_definitions(-DGZ_HEADERS)
  message("Compiling against Gazebo Garden")

else()
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
  set(IGN_RENDERING_VER ${ignition-gazebo6_VERSION_MAJOR})
  find_package(ignition-plugin1 REQUIRED)
  set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
  set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
  set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)
  set(GZ_RENDERING ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER})
  message("Compiling against Gazebo Fortress")
endif()

include_directories(${PCL_INCLUDE_DIRS})

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
  src/gazbeo_map_creator_ign.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "gazebo_map_creator_interface"
  "geometry_msgs"
  "rclcpp"
  pluginlib
)

ament_export_libraries(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${GZ_SIM}
  ${GZ_PLUGIN}
  ${GZ_RENDERING}
  ${PCL_LIBRARIES}
)

install(PROGRAMS DESTINATION lib/${PROJECT_NAME}/ )
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib )
# Install Python executables
install(PROGRAMS
  scripts/request_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()