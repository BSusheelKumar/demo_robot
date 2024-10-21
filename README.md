# Demo Robot

In this Task, I used Python to write scripts for switching maps, detecting obstacles, and stopping the robot. As I am proficient in Python, I opted to implement these features in Python. While I have limited exposure to C++, I have written some C++ code for ROS2 hardware control, and I believe that with practice, I can improve. In fact, I recently attempted to implement custom navigation plugins from the Nav2 stack but due to time constraints, I completed the task using Python. However, I am eager to learn how to create custom navigation plugins using the Behavior Tree (BT) Navigator.

## Key Features

1. **Map Switching**:  
   Map switching is handled through a service call to the `nav2_map_server`. The user provides input to switch maps, and the respective map is loaded. I had considered implementing an algorithm that would verify if the map aligns correctly with the respective Gazebo world, providing a prompt if the map is correct and a warning if it is not.

2. **Obstacle Detection**:  
   Obstacle detection is implemented using two methods: one based on LiDAR data and the other using image data.

3. **Challenges**:  
   Since I am using ROS2 Jazzy and Gazebo Harmonic, I was unable to utilize `twist_mux` for message multiplexing.

## Future Goals

I am keen to further explore custom navigation plugins using the Behavior Tree Navigator and improve my C++ skills to expand the range of solutions I can implement in ROS2.
