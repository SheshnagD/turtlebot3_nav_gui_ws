# ROS 2 TurtleBot3 Autonomous Navigation with GUI

This project demonstrates **autonomous waypoint navigation** for the TurtleBot3 robot in a **custom Gazebo world** using **ROS 2 Humble**.  
It integrates the **Navigation2 (Nav2)** stack with a **Tkinter-based GUI** that allows users to select one or multiple waypoints for the robot to visit autonomously.

---

## Overview

The system enables a simulated TurtleBot3 to:

- Navigate autonomously to predefined waypoints in a custom Gazebo environment.  
- Handle both single and multiple waypoint missions.  
- Display live navigation status on a simple GUI.  
- Return to the Home position automatically after completing navigation.

All components — **Gazebo world**, **Nav2 stack**, **Waypoint Manager**, and **GUI** — are launched together for smooth operation.

---

## Project Structure

```
turtlebot3_nav_gui_ws/
│
├── launch_all.sh                 # Unified launch script for all nodes
├── LICENSE                       # License file
├── README.md                     # Project documentation
│
└── src/
    ├── custom_turtlebot3_world/  # Custom Gazebo world, maps, and launch files
    ├── turtlebot3_nav_gui/       # Integration and launch utilities
    ├── waypoint_gui/             # GUI node (Tkinter-based)
    └── waypoint_manager/         # Navigation and waypoint management nodes
```

---

## Features

- Custom Gazebo world with 2D map (`custom_map.pgm`, `custom_map.yaml`)
- Autonomous navigation using the **Nav2** stack  
- Tkinter GUI with labeled buttons:
  - Station A, Station B, Station C, Station D, Docking Station, Home  
- Real-time navigation feedback: *Navigating*, *Reached*, *Failed*  
- Option to cancel navigation during execution  
- Automatic return to Home after mission completion  
- Single launch integration for Gazebo, Nav2, Waypoint Manager, and GUI  

---

## How to Run

### 1. Source ROS 2 Humble environment
```bash
source /opt/ros/humble/setup.bash
```

### 2. Build the workspace
```bash
cd ~/turtlebot3_nav_gui_ws
colcon build
```

### 3. Source the workspace
```bash
source install/setup.bash
```

### 4. Launch the full system
```bash
bash launch_all.sh
```

---

## Packages

| Package | Description |
|----------|-------------|
| **custom_turtlebot3_world** | Contains custom Gazebo world, map, and launch files |
| **turtlebot3_nav_gui** | Integrates and launches all components |
| **waypoint_gui** | Tkinter-based GUI for selecting and monitoring waypoints |
| **waypoint_manager** | Handles navigation goals and communicates with Nav2 |

---

## Demonstration Videos

- **Single Waypoint Navigation:** [Watch Here](https://drive.google.com/file/d/1cdENHyRTSt0GeGG20OROVQWUftz5Fwwp/view?usp=sharing)  
  *(Robot navigates from Home to one selected waypoint and returns Home.)*

- **Multiple Waypoints Navigation:** [Watch Here](https://drive.google.com/file/d/1SZyTktEvo7L3RrGDnyfN1M7WeRzi2bu6/view?usp=sharing)  
  *(Robot sequentially visits multiple waypoints and then returns Home.)*

---

## Technologies Used

- ROS 2 Humble (rclpy, Nav2, AMCL, map_server)  
- Gazebo (TurtleBot3 Simulation)  
- Tkinter (Python GUI)  
- Python 3.10+  
- RViz2 (for visualization)  

---

## License

This project is licensed under the **MIT License**.  
See the [LICENSE](LICENSE) file for details.

---

## Author

**Name:** Sheshnag D  
**Project:** TurtleBot3 Autonomous Navigation with GUI  
**Platform:** ROS 2 Humble + Gazebo Simulation  
**Date:** November 2025  

---
