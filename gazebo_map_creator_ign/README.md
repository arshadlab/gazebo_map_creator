IGN Gazebo/GZ plugin place holder (Work in Progress)
* System Plugin skeleton code with service creation and callback.
* Need to find equivalent of Gazebo Classic RayShape functionality in IGN
* Currently build disabled by placing COLCON_IGNORE file in package root directory.
* Test .world file in world directory

# IGN Gazebo (Gazebo 6)
 * Install dependencies
   rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
 * export GZ_VERSION=fortress
 * colcon build
 * export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=<path to plugin .so file>
 * ign gazebo -v 2  empty.world

# GZ SIM (Gazebo 7)
 * Install dependencies
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
 * export GZ_VERSION=garden
 * colcon build
 * export GZ_SIM_SYSTEM_PLUGIN_PATH=<path to plugin .so file>
 * gz sim -v 2  empty.world
