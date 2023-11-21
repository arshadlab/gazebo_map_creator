# ROS2 Gazebo World 2D/3D Map Generator
For realistic navigation utilizing Nav2, maps are typically produced employing SLAM methodologies, which extends to simulations in Gazebo as well. This ensures the incorporation of SLAM-generated maps in the path planning process, thereby facilitating a more accurate verification and behaviour in simulated environments. However, there are instances where the primary focus lies solely on the Nav2 path planning module. In such scenarios, the requirement shifts towards synthetic, yet precise, maps that can be utilized for development, testing in conjunction with simulations, and at times, actual deployment too.

Creating maps for ROS2 Navigation 2 (Nav2) in Gazebo has traditionally been a challenging task. The prevailing method involved employing Simultaneous Localization and Mapping (SLAM) and navigating robots through the environment to create these maps. For ROS1 Kinetic, a plugin was developed to mitigate this issue, enabling map creation without relying on SLAM. However, this solution was limited to ROS1 and generated 2D maps only.

This git repository presents a Gazebo 11 (Classic) plugin, interfaced with a ROS2 service, specifically develop to generate 2D (.pgm) and 3D maps (Point Cloud) directly from the Gazebo world.


Tested on ROS2 (Foxy, Humble & Iron) with Gazebo 11 (Classic).

Some work done for IGN gazebo support.  Partial code in gazebo_map_creator_ign directory (build disabled).

Guide:

https://medium.com/@arshad.mehmood/ros2-gazebo-world-map-generator-a103b510a7e5

### Dependencies
```
sudo apt-get install libboost-dev libpcl-dev
```

### Build
```
source /opt/ros/<humble|foxy>/setup.bash
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/arshadlab/gazebo_map_creator.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ./install/setup.bash
```
### Run

Console A
```
$ source ./install/setup.bash
$ gazebo -s libgazebo_map_creator.so myworld.world
```
Console B
```
$ source ./install/setup.bash
$ ros2 run gazebo_map_creator request_map.py -c '(-4.8,-4.5,0.03)(4.8,4.5,8.0)' -r 0.01 -f $PWD/map
$ ls
map.pcd map.bt map.pgm  map.png  map.yaml
```
### Viewing Results
```
# Open 2D pgm map.
# sudo apt-get install xdg-utils
xdg_open map.pgm

# Open 3D pcd point cloud map
# sudo apt-get install pcl-tools

pcl_viewer map.pcd
```

# Output samples
![image](https://github.com/arshadlab/gazebo_map_creator/assets/85929438/55abee71-e9ac-4a64-a159-e7b8dd059cd5)

![aws](https://github.com/arshadlab/gazebo_map_creator/assets/85929438/f68d92ec-ff45-4faa-979c-e96d3c585e20)

![image](https://github.com/arshadlab/gazebo_map_creator/assets/85929438/8901e692-9904-48a5-9cb2-174ba6c9c032)

![image](https://github.com/arshadlab/gazebo_map_creator/assets/85929438/1b3e9466-50f0-4b56-8d10-3a18ef8f0b5d)

![image](https://github.com/arshadlab/gazebo_map_creator/assets/85929438/eb1a6384-3332-43fe-a506-f98c9b0eca2a)


  
# Acknowledgement
Inspiration came from [Gazebo Custom Messages](https://gazebosim.org/wiki/Tutorials/1.9/custom_messages)
