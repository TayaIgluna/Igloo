#include "MoveRobot.h"
#include "Utils.h"
#include <boost/bind.hpp>

MoveRobot* MoveRobot::me = NULL;

MoveRobot::MoveRobot(ros::NodeHandle &n, float frequency):
  _n(n),
  _loopRate(frequency)
{

  ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveRobot::init() 
{
  me = this;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);
  _firstRobotPose = false;

  _stop = false;
  _toolOffsetFromEE = 0.0f;
  _msgCommand.data.resize(10);
  for(int k = 0; k < 10 ;k++)
  {
    _msgCommand.data[k] = 0.0f;
  }

  _subRobotPose = _n.subscribe<geometry_msgs::Pose>("/iiwa/ee_pose", 1,&MoveRobot::updateRobotPose,this,ros::TransportHints().reliable().tcpNoDelay());
  // _pubCommand = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/PassiveDS/command", 1);
  _pubCommand = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/DSImpedance/command", 1);

  signal(SIGINT,MoveRobot::stopNode);

  if (_n.ok())
  { 
    // Wait for callback to be called
    ros::spinOnce();
    ROS_INFO("The ros node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void MoveRobot::run() {

  while (!_stop) 
  {
    if(_firstRobotPose)
    {

      _mutex.lock();

      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);

  publishData();

  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void MoveRobot::stopNode(int sig)
{
  me->_stop = true;
}


void MoveRobot::setDesiredPose(Eigen::Vector3f desiredPosition, Eigen::Vector4f desiredQuaternion) 
{
  _xd = desiredPosition;
  _qd = desiredQuaternion;
  std::cerr << "[MoveRobot]: Desired position: " << _xd.transpose() << std::endl;
}


void MoveRobot::computeCommand() 
{
  // Compute desired velocity
  _vd = 3.0f*(_xd-_x);

  if(_vd.norm()>0.3f)
  {
    _vd *= 0.3f/_vd.norm();
  }
  
  // Compute desired angular velocity
  _omegad.setConstant(0.0f);

  std::cerr << "[MoveRobot]: " << "Position: " << _x.transpose() << " Quaternion: " << _q.transpose() << std::endl;

}


void MoveRobot::publishData()
{
  // Publish desired twist (passive ds controller)
  for(int k = 0; k < 3; k++)
  {
    _msgCommand.data[k]  = _vd(k);
    _msgCommand.data[k+3]  = _omegad(k);
  }
  for(int k = 0; k < 4; k++)
  {
    _msgCommand.data[k+6]  = _qd(k);
  }

  _pubCommand.publish(_msgCommand);
}


void MoveRobot::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  // Update end effecotr pose (position+orientation)
  _x << msg->position.x, msg->position.y, msg->position.z;
  _q << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb = Utils<float>::quaternionToRotationMatrix(_q);
  _x = _x+_toolOffsetFromEE*_wRb.col(2);

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
  }
}