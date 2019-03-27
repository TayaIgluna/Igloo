#include "ros/ros.h"
#include "MoveRobot.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "move_robot");

  Eigen::Vector3f desiredPosition;

  // Initialize desired position
  desiredPosition.setConstant(0.0f);

  // Check if desired position is specified with the command line
  if(argc == 4)
  {
    for(int k = 0; k < 3; k++)
    {
      desiredPosition(k) = atof(argv[k+1]);
    }
  }
  else
  {
    return 0;
  }

  ros::NodeHandle n;
  float frequency = 200.0f;

  MoveRobot moveRobot(n,frequency);

  if (!moveRobot.init()) 
  {
    return -1;
  }
  else
  {
    moveRobot.setDesiredPose(desiredPosition);
    moveRobot.run();
  }

  return 0;

}
