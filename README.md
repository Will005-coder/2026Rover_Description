# 2026 Rover — Robot Description Package

This repository contains the **robot description** for the *2026 Boston University Mars Rover* — a ROS2-compatible package including URDF/XACRO files, meshes, launch configurations, and RViz visualization.

> ⚙️ This package provides the core assets needed to visualize, simulate, and integrate the 2026 Rover within ROS 2 workflows.

---

## 📁 Repository Structure

```text
2026Rover_Description/
├── config/             # Configuration files for SLAM and other tools
├── launch/             # ROS 2 launch files for robot description & visualization
│   ├── display.launch.py       # Launch RViz with robot model
│   ├── gazebo.launch.py        # Launch Gazebo simulator with robot
│   ├── controller.launch.py    # Controller configuration
│   └── online_async_launch.py  # SLAM async launch
├── meshes/             # 3D model files (STL / DAE / OBJ)
├── rviz/               # RViz configuration files
├── urdf/               # URDF and XACRO robot description files
├── CMakeLists.txt      # ROS build instructions
├── package.xml         # ROS package metadata
└── LICENSE             # MIT License
```

**Note:** `build/`, `install/`, and `log/` directories are generated during build and should not be committed.

---

## 🚀 Features

This package enables:

- **Robot visualization** in RViz  
- **URDF/XACRO-based robot modeling** with proper link and joint definitions  
- **Gazebo simulation** with Ignition (ign_gazebo)
- **SLAM integration** with slam_toolbox for localization and mapping
- **Gazebo/ROS 2 bridge** for topic communication between simulation and ROS
- Launch files for quick startup  
- Easy integration as a dependency for **simulation, planning, and control** packages

---

## 📌 Prerequisites

- **ROS 2 Humble** (or compatible distribution)
- A configured ROS 2 workspace
- `colcon` build system
- Required ROS packages (see installation section)

---

## 🛠️ Installation

### Step 1: Clone the repository

```bash
git clone https://github.com/Will005-coder/2026Rover_Description.git
cd 2026Rover_Description
```

### Step 2: Install dependencies

First, ensure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then install the required ROS packages:

```bash
sudo apt-get update
sudo apt-get install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-slam-toolbox \
  ros-humble-nav2-common
```

### Step 3: Clean and build the workspace

```bash
# Remove any previous build artifacts
rm -rf build/ install/ log/

# Build the package
colcon build
```

### Step 4: Source the workspace

```bash
source install/setup.bash
```

---

## ▶️ Running the Rover Description

### Option 1: Visualize in RViz Only (Recommended for first-time setup)

```bash
ros2 launch urdf_description display.launch.py
```

This will:
- Load the robot model from URDF
- Display it in RViz
- Allow you to manually move joints using the joint state publisher

### Option 2: Simulate in Gazebo with RViz

```bash
ros2 launch urdf_description gazebo.launch.py
```

This will:
- Start Ignition Gazebo simulator
- Load the rover model
- Run SLAM for localization and mapping
- Display in RViz with simulation time synchronized
- Launch the bridge between Gazebo and ROS 2 topics

**Note:** The first launch may take 30-60 seconds to fully initialize.

---

## 🔧 Troubleshooting

### `No module named 'nav2_common'`
**Solution:** Make sure `nav2-common` is installed:
```bash
sudo apt-get install ros-humble-nav2-common
```

### `package 'joint_state_publisher_gui' not found`
**Solution:** The package uses `joint_state_publisher`, not `joint_state_publisher_gui`. If you need the GUI, install:
```bash
sudo apt-get install ros-humble-joint-state-publisher
```

### Build fails with merge conflict markers
**Solution:** The repository may have unresolved merge conflicts in `launch/gazebo.launch.py` or `CMakeLists.txt`. Contact the repository maintainer or manually resolve by removing conflict markers.

### Gazebo or RViz windows don't appear
**Solution:** 
- Ensure your display is properly configured (test with `echo $DISPLAY`)
- If using SSH/remote connection, use X11 forwarding: `ssh -X user@host`
- Try launching with debug output: `ros2 launch urdf_description display.launch.py --log-level DEBUG`

---

## 📚 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo Documentation](https://gazebosim.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

## 📄 License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.

---

## 📝 Contributing

To report issues or suggest improvements, please open an issue on the GitHub repository.
