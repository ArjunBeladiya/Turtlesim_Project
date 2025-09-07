# Turtlesim Collision Avoidance Project

This project demonstrates a simple **collision avoidance algorithm** using the **ROS2 Turtlesim** simulator.  
Two turtles move randomly inside the Turtlesim playground while avoiding:
- **walls** (boundaries of the simulator)
- **other turtle** (using Euclidean distance check)

The project was built to practice **ROS2 publishers, subscribers, custom nodes, and launch files**, and can serve as an entry-level robotics simulation project.

# Repo Structure:
```
Turtlesim_Project/
 ─ turtlesim_collision_avoidance/
   ─ launch/
     ─ collision_avoidance.launch.py   # Launch file for spawning turtles & running nodes
   ─ turtlesim_collision_avoidance/
     ─ random_mover.py                 # Node controlling random movement with collision avoidance
   ─ package.xml                       # ROS2 package metadata
   ─ setup.py                          # Python package setup 
   ─ resource/                         # Package resources
 ─ README.md
```
