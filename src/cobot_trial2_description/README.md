# Ultramotiv Assignment - ROS2 Cobot Simulation

This repository contains a ROS2 package for collaborative robot (cobot) simulation and kinematics analysis. The package includes Gazebo simulation, RViz visualization, and forward/inverse kinematics implementations.

## Package Overview

The `cobot_trial2_description` package provides:
- Robot description files (URDF/XACRO)
- Gazebo simulation environment
- RViz visualization configuration
- Forward and inverse kinematics algorithms

## Prerequisites

Before running this package, ensure you have the following installed:

- ROS2 (Humble/Iron/Rolling)
- Gazebo Classic or Gazebo Fortress/Garden
- RViz2
- Python 3.8+

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
- Provide interactive markers for robot control

### Kinematics Analysis

After launching both Gazebo and RViz, you can run the kinematics analysis scripts located in the `src` directory:

#### Forward Kinematics

To calculate the end-effector pose from joint angles:

```bash
cd src
python3 forward_kinematics.py
```

#### Inverse Kinematics

To calculate joint angles from desired end-effector pose:

```bash
cd src
python3 inverse_kinematics.py
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
│   ├── forward_kinematics.py
│   ├── inverse_kinematics.py
│   └── [other kinematics utilities]
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
   cd src
   python3 forward_kinematics.py
   # or
   python3 inverse_kinematics.py
   ```

## Features

- **Gazebo Integration**: Full physics simulation with collision detection
- **RViz Visualization**: Real-time robot state visualization
- **Forward Kinematics**: Calculate end-effector position from joint states
- **Inverse Kinematics**: Calculate joint angles for desired end-effector pose
- **Interactive Control**: Joint control through RViz or Gazebo interfaces

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

## License

This project is part of the Ultramotiv assignment. Please refer to the course guidelines for usage permissions.

## Contact

For questions or issues related to this assignment, please contact the course instructor or create an issue in this repository.

---

**Note**: Both launch files (gazebo_simulation.launch.py and display.launch.py) must be running simultaneously for full functionality. The kinematics scripts in the `src` directory provide additional analysis capabilities once the simulation environment is active.