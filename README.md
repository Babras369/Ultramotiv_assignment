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


# 🌟 My Implementation Methodology 🌟

## Phase 1: Robot Model Development and Integration

Create robotic arm model in Fusion 360. Make sure the joints have limits and links are properly named. Then use urdf2ros2 converter to create your ROS 2 compatible package.
- This package will help you convert fusion model to urdf [fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2.git).
- **OR** 
- Download the package of robotic arm from GitHub. Again it has to be compatible with ROS 2 Humble branch.

## Phase 2: MoveIt 2 Configuration and Setup

Install MoveIt 2 for ROS 2 Humble
Add all required dependencies listed in the README document
Optional plugins that might help:
```
sudo apt install ros-humble-moveit-common ros-humble-moveit-ros-planning-interface ros-humble-moveit-visual-tools
```

Run below command to start the setup assistant:
```
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
Once open, fill the required info and get the package in same directory as your current package

## Phase 3: Controller Configuration and Integration

Configure important files ros2_controllers.yaml and moveit_controllers.yaml.
These two files will help Gazebo understand the joints and controllers, and RViz MoveIt planner to send appropriate commands
Verify controller status using:
```
ros2 control list_controllers
```
## Phase 4: System Integration and Communication Pipeline

- The overall pipeline works like below:

    You move the robot in RViz and press execute, the motion planner like OMPL will plan the robot joints and send the command to appropriate controller (check the controller name and state to make sure it's active)
    Once the commands are received by the controller it will move the robot in Gazebo. Here the hardware plugins that are mentioned in the gazebo.xacro file will be responsible to view robot model in RViz, and joint_state_broadcaster will be responsible to tell the RViz current robot state through robot_state_publisher.
    Once the robot moves to its final position in Gazebo and RViz it will show log message in terminal as completed.
    **And voilà, you have successfully simulated the robotic arm using Gazebo and RViz.**

## phase 5: Object detection
- ### YOLOv8 Object Detection Training with Roboflow 
 - Hardware Requirements

   - GPU: NVIDIA GPU with CUDA support (recommended: RTX 3060 or higher)
   - RAM: Minimum 8GB, recommended 16GB+
   - Storage: At least 10GB free space for datasets and models

 - Software Requirements

   - Python3/ Python3.10(Stable)
   - CUDA 11.8+ and cuDNN (for GPU acceleration)
   - Git

 - Install Required Packages
   ```
   - pip install ultralytics
   - pip install roboflow 
   ```
 - Create New Project

   - Choose "Object Detection"
   - Set project name and description
   - Upload Images

 - Annotation Process

    - Manual Annotation
    - Use Roboflow's annotation tool
    - Draw bounding boxes around objects
    - Label each object with appropriate class name
    - Auto-Annotation (Optional)

 - Dataset Augmentation

    - Rotation: ±15 degrees
    - Brightness: ±25%
    - Exposure: ±25%
    - Blur: Up to 2.5px
    - Noise: Up to 5% of pixels
    - Flip: Horizontal only (if applicable)

 - Generate Dataset Version

    - Configure train/validation/test split (70/15/15)
    - Apply preprocessing and augmentation
    - Generate dataset version
    - Download in YOLOv8 format
                    
# For training the data set follow below file instructions:
 - yolov8.ipynb : ( /home/lenovo/ultramotive_ws/src/cobot_trial2_description/yolov8.ipynb )

# Demo video: Robotic arm controlled via Forward_kinamatics and Inverse_kinamatics 
https://github.com/user-attachments/assets/e4460ffe-7e6f-4b52-b301-fea9ec5767a3

# Demo video: Object detection for 3 different objects
https://github.com/user-attachments/assets/ec136a25-3e4d-4b66-906c-1a60cae641ee

# Demo Photo: Object detection using depth camera
![Image](https://github.com/user-attachments/assets/3686a4df-cfb7-49f6-be22-8ed0743c2ff4)
