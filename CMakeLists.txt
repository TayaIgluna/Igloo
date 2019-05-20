cmake_minimum_required(VERSION 2.8.3)

project(igluna)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  dynamic_reconfigure
  utils
  grasp_interface
)

find_package (Eigen3 REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure utils grasp_interface
  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(move_robot src/moveRobot_node.cpp src/MoveRobot.cpp)
target_link_libraries(move_robot ${catkin_LIBRARIES})
add_dependencies(move_robot ${PROJECT_NAME}_gencpp)

add_executable(move_robot_with_gripper src/moveRobotWithGripper_node.cpp src/MoveRobotWithGripper.cpp)
target_link_libraries(move_robot_with_gripper ${catkin_LIBRARIES})
add_dependencies(move_robot_with_gripper ${PROJECT_NAME}_gencpp)


## Declare a cpp library
FILE(GLOB SRCS src/MoveRobot.cpp)
FILE(GLOB INCS include/MoveRobot.h)


add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})