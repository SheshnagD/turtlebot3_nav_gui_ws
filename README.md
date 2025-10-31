# turtlebot3_nav_gui_ws
TurtleBot3 Navigation Project using ROS 2 Humble and Gazebo


# 🧭 ROS 2 TurtleBot3 Autonomous Navigation with GUI

This project demonstrates **autonomous waypoint navigation** for the TurtleBot3 robot in a **custom Gazebo world** using **ROS 2 Humble**.  
It integrates the **Navigation2 (Nav2)** stack with a **Tkinter-based GUI**, allowing users to select one or multiple waypoints for the robot to visit autonomously.

---

## 🧩 Overview

The system enables a simulated TurtleBot3 to:

- Navigate autonomously to predefined waypoints in a custom Gazebo environment.  
- Handle both single and multiple waypoint missions.  
- Display live navigation status on a Tkinter-based GUI.  
- Return to the Home position automatically after completing navigation.  

All components — **Gazebo world**, **Nav2 stack**, **Waypoint Manager**, and **GUI** — are launched together for seamless operation.

---

## 📁 Project Structure

turtlebot3_nav_gui_ws/
│
├── launch_all.sh # Unified launch script for all nodes
├── LICENSE # License file (if applicable)
├── README.md # Project documentation
│
└── src/
├── custom_turtlebot3_world/ # Custom Gazebo world, maps, and launch files
├── turtlebot3_nav_gui/ # Integration and launch utilities
├── waypoint_gui/ # GUI node (Tkinter-based)
└── waypoint_manager/ # Navigation and waypoint management nodes


---

## ⚙️ Features

- 🗺️ **Custom Gazebo world** with 2D occupancy map (`custom_map.pgm`, `custom_map.yaml`).
- 🧭 **Autonomous navigation** using the **Nav2 stack**.
- 🖥️ **Tkinter GUI** with labeled buttons for:
  - Station A  
  - Station B  
  - Station C  
  - Station D  
  - Docking Station  
  - Home
- 📊 **Real-time status feedback**:
  - `Navigating`, `Reached`, or `Failed`.
- ⏹️ **Cancel navigation** during execution.
- 🔁 **Automatic return to Home** after all waypoints are visited.
- 🚀 **Single launch integration** for Gazebo, Nav2, Waypoint Manager, and GUI.

---

## 🚀 How to Run

### 1. Source ROS 2 Humble environment
```bash
source /opt/ros/humble/setup.bash
2. Build the workspace
bash
Copy code
cd ~/turtlebot3_nav_gui_ws
colcon build
3. Source the workspace
bash
Copy code
source install/setup.bash
4. Launch the complete system
bash
Copy code
bash launch_all.sh
🧠 Packages Description
Package	Description
custom_turtlebot3_world	Contains custom Gazebo world, map, and launch files.
turtlebot3_nav_gui	Manages overall integration and system launch configuration.
waypoint_gui	Tkinter-based GUI for waypoint selection and monitoring.
waypoint_manager	Handles navigation goals and communicates with Nav2 action server.

🎥 Demonstration Videos
Single Waypoint Navigation: [Google Drive Link Here]
Robot navigates to a single selected waypoint and returns Home.

Multiple Waypoints Navigation: [Google Drive Link Here]
Robot visits multiple waypoints sequentially and then returns Home.

🧰 Technologies Used
ROS 2 Humble (rclpy, Nav2, AMCL, map_server)

Gazebo (TurtleBot3 Simulation)

Tkinter (Python GUI)

Python 3.10+

RViz2 (for visualization and debugging)

📜 License
This project is released under the MIT License.
See the LICENSE file for details.

👤 Author
Name: Sheshnag D
Project: TurtleBot3 Autonomous Navigation with GUI
Platform: ROS 2 Humble + Gazebo Simulation
Date: November 2025
