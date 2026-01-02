# 2026 Rover — Robot Description Package

This repository contains the **robot description** for the *2026 Rover* — a ROS-compatible model including its URDF, meshes, launch configurations, and RViz visualizations.

> ⚙️ This package provides the core description needed to simulate, visualize, and integrate the 2026 Rover robot within ROS workflows.

---

## 📁 Repository Structure

2026Rover_Description/
├── build/ # ROS2/Catkin build output
├── install/ # Install setup from build
├── launch/ # Launch files to start robot description & visualization
├── log/ # ROS logging output
├── meshes/ # 3D model files for robot links (STL/Collada/OBJ)
├── rviz/ # RViz configuration for robot visualization
├── urdf/ # URDF/XACRO robot description files
├── CMakeLists.txt # Build instructions for ROS
├── package.xml # ROS package metadata
├── test.xacro # (Example) included XACRO macro file
└── LICENSE # MIT License


---

## 🚀 Features

This package enables:

- **Robot visualization** in RViz  
- **URDF/XACRO-based robot model** with proper link/joint definitions  
- **Launch support** to start descriptions and visualizers quickly  
- Use as a dependency for simulation, planning, or control packages

---

## 📌 Prerequisites

This package is designed for use with **ROS 2** (e.g., Humble, Iron, or later) — make sure you have a working ROS installation and workspace setup.

---

## 🛠️ Installation

1. Clone this repo into your ROS workspace src directory:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/bakp22/2026Rover_Description.git

2. Build it
   cd ~/ros2_ws
   colcon build --symlink-install

3. Source the workspace
  ```bash
   source install/setup.bash

4. Visualize in RVIZ
  ```bash
    ros2 launch 2026Rover_Description display.launch.py


