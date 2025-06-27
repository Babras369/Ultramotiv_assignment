# ROS2 Cobot Simulation

This repository contains a ROS2 package for collaborative robot (cobot) simulation and kinematics analysis. The package includes Gazebo simulation, RViz visualization, and forward/inverse kinematics implementations.

## Package Overview

The `cobot_trial2_description` package provides:
- Robot description files (URDF/XACRO)
- Gazebo simulation environment
- RViz visualization configuration
- Forward and inverse kinematics algorithms

## Prerequisites

Before running this package, ensure you have the following installed:

- ROS2 (Humble)
- Gazebo Classic 
- RViz2
- Python3

### Install below dependences 
```bash
sudo apt-get update
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2_controllers ros-${ROS_DISTRO}-gazebo-ros2-control
```
## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Babras369/Ultramotiv_assignment.git
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Launch Simulation and Visualization

This package requires **two launch files** to be executed for complete functionality:

#### 1. Launch Gazebo Simulation

Start the Gazebo simulation environment with the cobot model:

```bash
ros2 launch cobot_trial2_description gazebo_simulation.launch.py
```

This will:
- Load the robot model in Gazebo
- Start the physics simulation
- Initialize robot controllers

#### 2. Launch RViz Display

In a **separate terminal**, launch the RViz visualization:

```bash
ros2 launch cobot_trial2_description display.launch.py
```

This will:
- Open RViz2 with pre-configured robot visualization
- Display robot state and joint information
- Provide full configured robot to control

### Kinematics Analysis

After launching both Gazebo and RViz, you can run the kinematics analysis scripts located in the `src` directory:

#### Forward Kinematics

To calculate the end-effector pose from joint angles:
 - This will show the end-effector pose in the terminal 

```bash
ros2 run cobot_trial2_description fk_node
```

#### Inverse Kinematics

To calculate joint angles from desired end-effector pose:
 - This will show the joint angles in the terminal with joint names respectively.

```bash
ros2 run cobot_trial2_description ik_node
```

## File Structure

```
Ultramotiv_assignment/
├── cobot_trial2_description/
│   ├── launch/
│   │   ├── gazebo_simulation.launch.py
│   │   └── display.launch.py
│   ├── urdf/
│   │   └── [robot description files]
│   ├── meshes/
│   │   └── [3D model files]
│   ├── config/
│   │   └── [configuration files]
│   └── CMakeLists.txt
├── src/
│   ├── fk_node.cpp
│   └── ik_node.cpp
└── README.md
```

## Quick Start Guide

1. **Terminal 1**: Launch Gazebo simulation
   ```bash
   ros2 launch cobot_trial2_description gazebo_simulation.launch.py
   ```

2. **Terminal 2**: Launch RViz display
   ```bash
   ros2 launch cobot_trial2_description display.launch.py
   ```

3. **Terminal 3**: Run kinematics analysis (optional)
   ```bash
   ros2 run cobot_trial2_description fk_node
   # or
   ros2 run cobot_trial2_description ik_node
   ```

## Features

- **Gazebo Integration**: Full physics simulation with object detection
- **RViz Visualization**: Real-time robot state visualization
- **Forward Kinematics**: Calculate end-effector position from joint states
- **Inverse Kinematics**: Calculate joint angles for desired end-effector pose
- **Interactive Control**: Joint control through **"joint_trajectory_controller/joint_trajectory"** topic

## Troubleshooting

### Common Issues

1. **Launch files not found**: Ensure the package is built and sourced correctly
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select cobot_trial2_description
   source install/setup.bash
   ```

2. **Gazebo not starting**: Check if Gazebo is properly installed and configured
   ```bash
   gazebo --version
   ```

3. **RViz display issues**: Verify RViz2 installation and robot model loading
   ```bash
   ros2 topic echo /robot_description
   ```

### Dependencies

If you encounter missing dependencies, install them using:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

<div align="center"> 

🌟 Your Feedback is very valuable ! 🌟

</div>

### Help me Improve This Project 
 - Do leave a feedback on anything that can be improved .
 - Your experience , insights and suggestions are invaluable to making this simulation better!
 - Every contribution, no matter how small, makes a difference!

## Ways to Contribute:
    ⭐ Star this repository if you find it useful
    🐛 Report bugs or suggest features
    📖 Improve documentation
    🔧 Submit pull requests with enhancements
    💬 Share your experience using this simulation
---

## License

This project is part of the Ultramotiv assignment. Please refer to the first README file, where i have placed the assignemnt content.

## Contact
For questions or issues related to this assignment, please create an issue in this repository.

**Note**: Both launch files (gazebo_simulation.launch.py and display.launch.py) must be running simultaneously for full functionality. The kinematics scripts in the `src` directory provide additional analysis capabilities once the simulation environment is active.