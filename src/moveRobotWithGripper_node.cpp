#include "ros/ros.h"
#include "MoveRobotWithGripper.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "move_robot_with_gripper");

  ros::NodeHandle n;
  float frequency = 200.0f;

  MoveRobotWithGripper moveRobotWithGripper(n,frequency);

  if (!moveRobotWithGripper.init()) 
  {
    return -1;
  }
  else
  {
    moveRobotWithGripper.run();
  }

  return 0;

}
