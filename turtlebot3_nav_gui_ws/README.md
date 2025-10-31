# ROS 2 TurtleBot3 Autonomous Navigation with GUI

This project demonstrates autonomous waypoint navigation for the TurtleBot3 robot in a custom Gazebo world using **ROS 2 Humble**.  
It integrates the **Navigation2 (Nav2)** stack with a **Tkinter-based GUI** that allows users to select one or more waypoints for the robot to visit autonomously.

---

##  Overview

The system enables a simulated TurtleBot3 to:
- Navigate to predefined waypoints within a custom world.
- Handle both single and multiple waypoint missions.
- Display live navigation status on a simple GUI.
- Return to the Home position automatically after completing navigation.

All components — Gazebo world, navigation stack, GUI, and waypoint manager — are launched together through a single launch file for smooth integration.

---

##  Project Structure

turtlebot3_nav_gui_ws/
│
├── launch_all.sh
│
└── src/
├── custom_turtlebot3_world/ → Contains custom map and world files.
├── turtlebot3_nav_gui/ → Integration and launch utilities.
├── waypoint_gui/ → GUI node for waypoint selection.
└── waypoint_manager/ → Node managing navigation goals.



---

##  Features

- **Custom Gazebo world** with 2D map (`custom_map.pgm`, `custom_map.yaml`).
- **Tkinter GUI** with labeled buttons for:
  - Station A  
  - Station B  
  - Station C  
  - Station D  
  - Docking Station  
  - Home
- GUI options:
  - Select single or multiple waypoints.
  - Display navigation progress (`Navigating`, `Reached`, `Failed`).
  - Cancel navigation during execution.
- **Waypoint Manager Node**:
  - Sends goals to Nav2’s `NavigateToPose` action server.
  - Handles feedback, success, and failure cases.
  - Returns robot to Home after completing all selected goals.
- **Single Launch Integration**:
  - Starts Gazebo world, Nav2 stack, waypoint manager, and GUI together.

---

##  How to Run

1. Source your ROS 2 Humble environment:
   ```bash
    source /opt/ros/humble/setup.bash
2. Build the workspace:
    cd ~/turtlebot3_nav_gui_ws
    colcon build
3. Source the workspace:
    source install/setup.bash
4. Launch the full system:
    bash launch_all.sh


🎥 Demonstration Videos

Single Waypoint Navigation – https://drive.google.com/file/d/1cdENHyRTSt0GeGG20OROVQWUftz5Fwwp/view?usp=sharing
(Robot navigates to a HOME > single selected waypoint and returns Home.)

Multiple Waypoints Navigation – https://drive.google.com/file/d/1SZyTktEvo7L3RrGDnyfN1M7WeRzi2bu6/view?usp=sharing
(Robot sequentially visits multiple waypoints and then returns Home.)

🧠 Technologies Used

ROS 2 Humble (rclpy, Nav2, AMCL, map_server)

Gazebo (TurtleBot3 Simulation)

Tkinter (Python GUI)

Python 3.10

RViz2 for Visualization





