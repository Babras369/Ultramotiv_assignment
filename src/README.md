# Robotics Developer Assignment

## Objective:
Develop and simulate a complete 6-DOF robotic arm system in ROS 2 (Humble), using Gazebo, RViz, and
MoveIt 2, and implement motion planning and perception through kinematics and computer vision.

---

**1. Develop a 6-DOF Robotic Arm Simulation**

### Tools: 
Gazebo, RViz, MoveIt 2
### Description:
- Choose any 6-DOF robotic arm (e.g., UR5, UR10, Kinova Gen3, etc.)
- Simulate the arm in Gazebo
- Visualize it in RViz
- Configure MoveIt 2 for motion planning

---

**2. Write a ROS 2 Package for Kinematics Control**

### Description:
- Create a ROS 2 package or node
- Implement forward and inverse kinematics
- Publish/subscribe to appropriate topics or services for joint control
- Add a small test routine (e.g., move to given XYZ pose)

---
**3. Simulate Depth Camera and Apply Object Detection**

### Tools: 
Gazebo depth camera plugin, OpenCV, YOLOv5/YOLOv8 or any pretrained model
### Description:
- Add a simulated depth or RGB-D camera to the robot or scene in Gazebo
- Capture images and run object detection
- Display detection results in RViz or console

---
**NOTE**
    This package will give you a breif understanding of how robotic arm is simulated in gazebo and can be controlled 