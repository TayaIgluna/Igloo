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
  _msgCommand.data.resize(6);
  for(int k = 0; k < 6 ;k++)
  {
    _msgCommand.data[k] = 0.0f;
  }

  _pubCommand = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/CustomControllers/command", 1);

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

      receiveFrames();

      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      _mutex.unlock();
    }
    else
    {
      receiveFrames();
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


void MoveRobotWithGripper::receiveFrames()
{
  try
  { 
    //_lr.waitForTransform("/world", "/iiwa_link_ee", ros::Time(0), ros::Duration(3.0));
    _lr.lookupTransform("/world", "/iiwa_link_ee",ros::Time(0), _transform);
    _x << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
    _q << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
    _wRb =  Utils<float>::quaternionToRotationMatrix(_q); 
    _x+= _toolOffsetFromEE*_wRb.col(2);
    float angle;
    Eigen::Vector3f axis;
    Utils<float>::quaternionToAxisAngle(_q,axis,angle);
    _aa = angle*axis;
    //std::cerr << "Axis angle measured: " << _aa.transpose() << std::endl;
    
    if(_firstRobotPose==false)
    {
      _firstRobotPose = true;
      _qd = _q;
      _xd = _x;
      _aad = _aa;
      _vd.setConstant(0.0f);
    }

    //std::cerr << "Position: " << _x.transpose() << std::endl;
  } 
  catch (tf::TransformException ex)
  {
  }
}


void MoveRobotWithGripper::computeCommand() 
{
  _plants[0] << 0.395f, -0.105f, 0.789f;
  _plants[1] << 0.6f, -0.105f, 0.73f;
  _plants[2] << 0.445f, -0.258f, 0.789f;
  // Compute desired velocity$
  //_attractor[0] << 0.54f, 0.0f, 0.75f;
  _attractor[0]=_plants[2];
  _attractor[0](2)-=0.15f;
  //too low horizontal pick
  //_attractor[1] << 0.174f, -0.107f, 0.973f;
  //_attractor[1] << 0.395f, -0.105f, 0.789f; 
  _attractor[1]=_plants[2];
  _attractor[2]=_attractor[1];
  _attractor[2](2)-=0.1f;
  _attractor[3] << 0.81f, 0.0f, 0.68f;
  //_attractor[2] << 0.3f, -0.2f, 0.75f;
  //attrac 4 same as 3 pour hor
  _attractor[4] << 0.746f, -0.042f, 0.638f;

  //_quat[0] << 0.69f, 0.022f, 0.71f, 0.0f; 
  _quat[1] << 0.999f, -0.02f, 0.027f, 0.027f; 
  _quat[0]=_quat[1];
  _quat[2]=_quat[1];
  _quat[3]=_quat[1];
  _quat[4]<< 0.39f, 0.87f, -0.03f, 0.286f;
  // too low
  //_quat[1] << 0.693f, -0.006f, 0.721f, -0.01f; 
  //_quat[2] << 0.71f, -0.001f, 0.704f,-0.03f;
  //_quat[2] << 0.68f, 0.01f, 0.73f,0.0f; goods
  //_quat[3] << 0.672f, 0.005f, 0.740f, -0.003f; 
  //_quat[4] << 0.075f, 0.61f, 0.248f, 0.748f;
  //_quat[4] << 0.206f, 0.646f, 0.182f, 0.712f;
  _qd=_quat[_id];
  _qd.normalize();

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
        _id = 1;
        _reached = false;
        _first = false; 
        std::cerr << "a" << std::endl;
      }
    }
  }
  else if(_reached && _id == 1)
  { 
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
        //_id = 2;
        _reached = false;
        _first = false; 
        std::cerr << "b" << std::endl;
        _id=2;
      }
    }
  }
  else if(_reached && _id == 2)
  { 
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec();  
      //_gripper.fullOpen(); 
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        _reached = false;
        _first = false; 
        std::cerr << "c" << std::endl;
        _id=3;
      }
    }
  }
  else if(_reached && _id == 3)
  { 
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec(); 
      //_gripper.fullOpen(); 
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        _id = 4;
        _reached = false;
        _first = false; 
        std::cerr << "d" << std::endl;
      }
    }
  }
    else if(_reached && _id == 4)
  { 
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec(); 
      //_gripper.fullOpen(); 
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        _id = 0;
        _reached = false;
        _first = false; 
        std::cerr << "d" << std::endl;
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

  float angle;
  Eigen::Vector3f axis;
  Utils<float>::quaternionToAxisAngle(_qd,axis,angle);
  _aad = angle*axis;
  //std::cerr << "Axis angle desired: " << _aad.transpose() << std::endl;

  //std::cerr << "[MoveRobotWithGripper]: " << "Position: " << _x.transpose() << " Quaternion: " << _q.transpose() << std::endl;

}


void MoveRobotWithGripper::publishData()
{
  // Publish desired twist (passive ds controller)
  for(int k = 0; k < 3; k++)
  {
    _msgCommand.data[k]  = _aad(k);
    _msgCommand.data[k+3]  = _vd(k);
  }
  _pubCommand.publish(_msgCommand);
}
