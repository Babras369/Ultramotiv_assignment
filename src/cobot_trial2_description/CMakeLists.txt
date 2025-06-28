cmake_minimum_required(VERSION 3.5)
project(cobot_trial2_description)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)    
find_package(cv_bridge REQUIRED)    

ament_python_install_package(${PROJECT_NAME})

# Declare executable: fk_node
add_executable(fk_node src/fk_node.cpp)
add_executable(ik_node src/ik_node.cpp)

# Link dependencies
ament_target_dependencies(fk_node rclcpp trajectory_msgs)
ament_target_dependencies(ik_node rclcpp trajectory_msgs)


# Include Eigen headers
target_include_directories(fk_node PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(ik_node PRIVATE ${EIGEN3_INCLUDE_DIR})

install(PROGRAMS
  scripts/yolo_object_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  fk_node
  ik_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories (URDF, meshes, config, launch)
install(
  DIRECTORY 
  launch
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
# Export package
ament_package()
