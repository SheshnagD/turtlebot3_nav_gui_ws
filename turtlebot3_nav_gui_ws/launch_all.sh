#!/bin/bash
# ==============================================
# 🐢 TurtleBot3 Full Launch Script (Simulation + Nav + GUI)
# ==============================================

# ---- CONFIG ----
WORKSPACE=~/turtlebot3_nav_gui_ws
MAP_PATH=$WORKSPACE/src/custom_turtlebot3_world/maps/custom_map.yaml

# ---- KILL OLD PROCESSES ----
echo "🔹 Killing existing ROS, Gazebo, and RViz processes..."
pkill -9 rviz2 2>/dev/null
pkill -9 ros2 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 nav2 2>/dev/null
echo "✅ Old processes terminated."

# ---- SOURCE ROS & WORKSPACE ----
echo "🔹 Sourcing ROS 2 and workspace..."
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash

# ---- Launch Simulation ----
echo "🚀 Launching Simulation World..."
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
source $WORKSPACE/install/setup.bash; \
ros2 launch custom_turtlebot3_world custom_world.launch.py; \
exec bash"

# ---- Launch Navigation ----
sleep 8  # Wait for simulation to initialize
echo "🧭 Launching Navigation..."
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
source $WORKSPACE/install/setup.bash; \
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$MAP_PATH; \
exec bash"

# ---- Wait for map to be available ----
echo "⏳ Waiting for /map topic..."
until ros2 topic list | grep -q '/map'; do
    sleep 1
done
echo "✅ Map topic detected."

# ---- Launch GUI ----
sleep 3  # Give navigation a few more seconds
echo "🖥️ Launching Waypoint GUI..."
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
source $WORKSPACE/install/setup.bash; \
ros2 run waypoint_gui gui_node; \
exec bash"

echo "✅ All systems launched!"

