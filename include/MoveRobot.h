#ifndef __MOVE_ROBOT_H__
#define __MOVE_ROBOT_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>

class MoveRobot 
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
		Eigen::Vector4f _q;
		Eigen::Matrix3f _wRb;
		Eigen::Vector3f _omegad;
		Eigen::Vector3f _vd;
		bool _firstRobotPose;
		float _toolOffsetFromEE;
		tf::TransformListener _lr;
		tf::StampedTransform _transform;

		// Class variables
		std::mutex _mutex;

		bool _stop = false;
		static MoveRobot* me;

	public:
		MoveRobot(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		// Set desired joint angles
		void setDesiredPose(Eigen::Vector3f desiredPosition, Eigen::Vector4f desiredQuaternion);

	private:

		static void stopNode(int sig);

  		void computeCommand();
  
 	 	void publishData();

 	 	void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

};


#endif
