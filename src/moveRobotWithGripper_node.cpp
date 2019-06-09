#include "ros/ros.h"
#include "MoveRobotWithGripper.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "move_robot_with_gripper");

  ros::NodeHandle n;
  float frequency = 400.0f;

  MoveRobotWithGripper moveRobotWithGripper(n,frequency);
  //ros::Subscriber _subRobArm = n.subscribe("/robArm/cmd", 10, &MoveRobotWithGripper::callBackRob, &moveRobotWithGripper);

  if (!moveRobotWithGripper.init()) 
  {
    return -1;
  }
  else
  {
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
    moveRobotWithGripper.run();
  }

  return 0;

}
