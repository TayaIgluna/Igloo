#ifndef __MOVE_ROBOT_WITH_GRIPPER_H__
#define __MOVE_ROBOT_WITH_GRIPPER_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <grasp_interface/r2f_gripper_interface.h>


class MoveRobotWithGripper 
{

	private:
		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subRobotPose; // Subscribe to robot current pose
		ros::Publisher _pubCommand;    // Publish desired orientation

		std_msgs::Float64MultiArray _msgCommand;

		// Node variables
		Eigen::Vector3f _x;
		Eigen::Vector3f _xd;
		Eigen::Vector4f _qd;
		Eigen::Vector3f _aa;
		Eigen::Vector3f _aad;
		Eigen::Vector4f _q;
		Eigen::Matrix3f _wRb;
		Eigen::Vector3f _omegad;
		Eigen::Vector3f _vd;
		bool _firstRobotPose;
		bool _aero;
		bool _pick;
		float _toolOffsetFromEE;
		tf::TransformListener _lr;
		tf::StampedTransform _transform;

		double _reachedTime;

		Eigen::Vector3f _attractor[10];
		//Eigen::Vector3f _plants[8];
		Eigen::Vector4f _quat[10];
		int _id;
		bool _reached;

		// Class variables
		std::mutex _mutex;

		r2fGripperInterface _gripper;
		bool _first;

		bool _stop = false;
		static MoveRobotWithGripper* me;
		//const Eigen::Vector4f _quat[10]=Eigen::Vector4f::Constant(10, Eigen::Vector4f(0.999f, -0.02f, 0.027f, 0.027f));
		const Eigen::Vector3f _plants[3]={Eigen::Vector3f (0.42f, -0.148f, 0.83f), Eigen::Vector3f(0.6f, -0.105f, 0.73f), Eigen::Vector3f (0.445f, -0.258f, 0.789f)};
		/*  _plants[0] << 0.42f, -0.148f, 0.83f;
  _plants[1] << 0.6f, -0.105f, 0.73f;
  _plants[2] << 0.445f, -0.258f, 0.789f;
  _plants[3] << 0.6f, -0.235f, 0.789f;
  _plants[4] << 0.6f, -0.345f, 0.789f;
  _plants[5] << 0.073f, -0.331f, 0.852f;*/


	public:
		MoveRobotWithGripper(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

	private:

		static void stopNode(int sig);

		void receiveFrames();

  		void computeCommand();
  
 	 	void publishData();

};


#endif
