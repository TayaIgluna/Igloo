#include "MoveRobotWithGripper.h"
#include "Utils.h"
#include <boost/bind.hpp>

MoveRobotWithGripper* MoveRobotWithGripper::me = NULL;



MoveRobotWithGripper::MoveRobotWithGripper(ros::NodeHandle &n, float frequency):
  _n(n),
  _loopRate(frequency)
{

  ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
  //_subRobArm = _n.subscribe("/robArm/cmd", 10, &MoveRobotWithGripper::callBackRob);
}

void MoveRobotWithGripper::callBackRob( const growbot_msg::RobArm_cmd::ConstPtr& msg)
{
  ROS_INFO("I heard [%d]", msg->potID);
  _aero=msg->aero;
  _potID=msg->potID;
  _pick=msg->bringBack;
  _fini=false;
  _msgMoving.isMoving=true;
  _pubFini.publish(_msgMoving);
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
  _subRobArm = _n.subscribe<growbot_msg::RobArm_cmd>("/robArm/cmd", 10, &MoveRobotWithGripper::callBackRob, this);
  //sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &CustomEffortController::commandCB, this);
  //_aero=this->aero;
  _pubFini = _n.advertise<growbot_msg::RobArm_moving>("/robArm/moving", 10);
  _pubDispenser = _n.advertise<growbot_msg::Dispenser_cmd>("/dispenser/cmd",10);
  signal(SIGINT,MoveRobotWithGripper::stopNode);


  _gripper.activate();
  _gripper.setSpeed(-1);

   ros::spinOnce();
  _loopRate.sleep();

  _reached = false;
  _fini=true;
  
  _aero= false;
  _pick=true;
  _potID=1;

  
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

  /*growbot_msg::RobArm_cmdConstPtr msgs =ros::topic::waitForMessage<growbot_msg::RobArm_cmd>("/robArm/cmd", _n);
  _aero=msgs->aero;
  _potID=msgs->potID;
  _pick=msgs->bringBack;*/
  std::cerr<<"aa"<<std::endl;
  _fini=false;

  if(!_pick)
  {
    _id = 0;
  }
  else
  {
    _id=6;
  }

  while (!_stop)
  {
    if(_firstRobotPose && !_fini)
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
  } 
  catch (tf::TransformException ex)
  {
  }
}


void MoveRobotWithGripper::computeCommand() 
{

  // Compute desired velocity$

  _attractor[0] << 0.54f, 0.0f, 0.6f;
  _attractor[1]=_plants[_potID];
  _attractor[1](2)-=0.1f;

  _attractor[2]=_plants[_potID];
  //_attractor[2]=_plants[potID];
  _attractor[3]=_attractor[1];
  _attractor[3](2)-=0.17f;
  //good ones 4&5 aero
  if (_aero)
  {
    _attractor[4] << 0.761f, -0.13f, 0.59f;
    _attractor[5] << 0.741f, -0.059f, 0.503f;

  //laero suite
    _attractor[6]=_attractor[0];
    _attractor[7] << 0.50f, -0.44f, 0.724f;

    _attractor[8] << 0.55f, -0.44f, 0.724f;
  
    _attractor[9]=_attractor[7];
    _attractor[10]=_attractor[7];
    _attractor[10](2)-=0.05f;
    _attractor[11]=_plants[_potID];
    _attractor[11](2)-=0.22f;
    _attractor[12]=_attractor[2];
    _attractor[13]=_attractor[2];
    _attractor[13](2)-=0.1f;
    _attractor[14]=_attractor[0];
    //_quat[5]<< 0.42f, 0.87f, 0.014f, 0.015f;
  }
  else if(!_aero && !_pick)
  {
    _attractor[4] << 0.705f, 0.223f, 0.612f;
    _attractor[5] << 0.725f, 0.223f, 0.62f;
    _attractor[6]=_attractor[0];
    _attractor[7]=_attractor[0];
  }
  else if(!_aero && _pick)
  {
    _attractor[4] << 0.715f, 0.223f, 0.612f;
    _attractor[5] << 0.715f, 0.223f, 0.66f;
    _attractor[6] << 0.715f, 0.223f, 0.612f;
    _attractor[7] =_attractor[0];
  }
  if(_aero)
  {
    nb=13;
  }
  else
  {
    nb=7;
  }
  Eigen::Vector4f _quati (1.0f, 0.0f, 0.0f, 0.0f);
  if(_id==5 && _aero)
  {
    _qd<<0.42f, 0.87f, 0.014f, 0.015f;
  }
  else
  {
    _qd=_quati;
  }
  _qd.normalize();
  _vd = 4.0f*(_attractor[_id]-_x);

  if((_attractor[_id]-_x).norm()<0.05)
  {
    _reached = true;
  }
  else
  {
    _reached = false;
  }

  if(_reached)
  {
    if(!_first)
    {
      _first=true;
      _reachedTime = ros::Time::now().toSec();  
      //_gripper.fullOpen();
      if(_id==2 && !_pick)
      {
        _gripper.fullClose();
      }
      else if(_id==2 && _pick)
      {
        _gripper.fullOpen();
      }
      else if(_id==8 && _aero)
      {
        _pubDispenser.publish(_msgCom);
        //growbot_msg::Dispenser_movingConstPtr msgs =ros::topic::waitForMessage<growbot_msg::Dispenser_moving >("/dispenser/moving", _n);
        std::cerr<<"jiji"<<std::endl;
      }
      else if(_id==12)
      {
        _gripper.fullOpen();
      }

      if(_id==5 && !_aero)
      {
        if(!_pick)
        {
          _gripper.fullOpen();
        }
        else
        {
          _gripper.fullClose();
        }
      }
    }
    else
    {
      if(ros::Time::now().toSec()-_reachedTime>2)
      {
        if(!_pick)
        {
          if(_id!=nb && _id!=8)
          {
            _id = _id+1;
          }
          else if(_id==8)
          {
            while(ros::Time::now().toSec()-_reachedTime<4);
            _id=9;
          }
          else
          {
            _id=0;
            _msgMoving.isMoving=false;
            _pubFini.publish(_msgMoving);
            _fini=true;
            return;
          }
        }
        else
        {
          if(_id!=0)
          {
            _id=_id-1;
          }
          else
          {
            _id=nb;
            _msgMoving.isMoving=false;
            _pubFini.publish(_msgMoving);
            _fini=true;
            return;
          } 
        }
        _reached = false;
        _first = false; 
        std::cerr << _id << std::endl;
      }
    }
  }
  
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


