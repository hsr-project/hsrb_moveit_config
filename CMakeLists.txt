cmake_minimum_required(VERSION 3.10.2)
project(hsrb_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)

add_executable(moveit_constraints_demo example/moveit_constraints_demo.cpp)
target_include_directories(moveit_constraints_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_constraints_demo moveit_ros_planning_interface rclcpp)

add_executable(moveit_fk_demo example/moveit_fk_demo.cpp)
target_include_directories(moveit_fk_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_fk_demo moveit_ros_planning_interface rclcpp)

add_executable(moveit_gripper_demo example/moveit_gripper_demo.cpp)
target_include_directories(moveit_gripper_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_gripper_demo moveit_ros_planning_interface rclcpp)

add_executable(moveit_ik_demo example/moveit_ik_demo.cpp)
target_include_directories(moveit_ik_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_ik_demo moveit_ros_planning_interface rclcpp)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_joint_states_publisher.py DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS moveit_constraints_demo moveit_fk_demo moveit_gripper_demo moveit_ik_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_package()