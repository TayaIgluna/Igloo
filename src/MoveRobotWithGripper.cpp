#include "MoveRobotWithGripper.h"
#include "Utils.h"
#include <boost/bind.hpp>

MoveRobotWithGripper* MoveRobotWithGripper::me = NULL;

MoveRobotWithGripper::MoveRobotWithGripper(ros::NodeHandle &n, float frequency):
  _n(n),
  _loopRate(frequency)
{

  ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveRobotWithGripper::init() 
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

  _subRobotPose = _n.subscribe<geometry_msgs::Pose>("/iiwa/ee_pose", 1,&MoveRobotWithGripper::updateRobotPose,this,ros::TransportHints().reliable().tcpNoDelay());
  // _pubCommand = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/PassiveDS/command", 1);
  _pubCommand = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/DSImpedance/command", 1);

  signal(SIGINT,MoveRobotWithGripper::stopNode);

  _gripper.activate();
  _gripper.setSpeed(-1);

   ros::spinOnce();
  _loopRate.sleep();

  _id = 0;
  _reached = false;
  
  _first = false;

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


void MoveRobotWithGripper::run() {

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

  _gripper.fullOpen();

  ros::spinOnce();
  _loopRate.sleep();

  ros::Duration(2.0).sleep();

  _gripper.reset();
  
  ros::spinOnce();
  _loopRate.sleep();


  ros::shutdown();
}


void MoveRobotWithGripper::stopNode(int sig)
{
  me->_stop = true;
}



void MoveRobotWithGripper::computeCommand() 
{
  // Compute desired velocity$

  //_attractor[0]=_xd;
  _attractor[0](0)=0.54f;
  _attractor[0](1)=0.0f;
  _attractor[0](2)=0.75f;
  /*_quat[0]=_qd;
  _quat[0](0)=0.69f;
  _quat[0](1)=0.022f;
  _quat[0](2)=0.71f;
  _quat[0](3)=0.0f;*/
 
  _attractor[1] = _xd;
  _attractor[1](0)=0.42f;
  _attractor[1](1)=0.0f;
  _attractor[1](2)= 1.0f;
  _attractor[2] = _xd;
  _attractor[2](0)= 0.21f;
  _attractor[2](1) = -0.3f;
  _attractor[2](2) =0.7f;
  _attractor[3]= _xd;
  _attractor[3](1) -=0.4f;
  _attractor[3](0) -=0.2f;

  _vd = 3.0f*(_attractor[_id]-_x);

  if((_attractor[_id]-_x).norm()<0.05)
  {
    _reached = true;
  }
  else
  {
    _reached = false;
  }

  if(_reached && _id == 0)
  {
    //_id = 1;
    //_reached = false;
    
    //_gripper.fullClose();
    
    
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec();  
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        _id = 1;
        _reached = false;
        _first = false; 
        std::cerr << "a" << std::endl;
      }
    }
  }
  else if(_reached && _id == 1)
  {
    //_id = 0;
    //_reached =false;
    
    //_gripper.fullClose();
    Eigen::Vector4f quater (0.71f, 0.01f, 0.702f, -0.002f);
    quater.normalize();
    _qd=quater;
    
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec(); 
      _gripper.fullClose();  
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        _id = 2;
        _reached = false;
        _first = false; 
        std::cerr << "b" << std::endl;
      }
    }
  }
  else if(_reached && _id == 2)
  {
    //_id = 0;
    //_reached =false;
    
    //_gripper.fullOpen();
    Eigen::Vector4f quate (0.58f, 0.38f, 0.62f, -0.35f);
    quate.normalize();
    _qd=quate;
    
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec();  
      _gripper.fullOpen(); 
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        _id = 0;
        _reached = false;
        _first = false; 
        std::cerr << "c" << std::endl;
      }
    }
  }

  //std::cerr << (_attractor[_id]-_x).norm() << " " <<(int) _reached << " " << _id << " " << ros::Time::now().toSec()-_reachedTime <<std::endl;

  //_vd = 3.0f*(_xd-_x);

  if(_vd.norm()>0.3f)
  {
    _vd *= 0.3f/_vd.norm();
  }
  
  // Compute desired angular velocity
  _omegad.setConstant(0.0f);

  //std::cerr << "[MoveRobotWithGripper]: " << "Position: " << _x.transpose() << " Quaternion: " << _q.transpose() << std::endl;

}


void MoveRobotWithGripper::publishData()
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


void MoveRobotWithGripper::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  // Update end effecotr pose (position+orientation)
  _x << msg->position.x, msg->position.y, msg->position.z;
  _q << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb = Utils<float>::quaternionToRotationMatrix(_q);
  _x = _x+_toolOffsetFromEE*_wRb.col(2);

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
  }
}