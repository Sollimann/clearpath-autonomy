#pragma once

#include <string>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace jackal_tf2_broadcaster_ns {

class TransformBroadcaster
{	

public:

	// explicit contructor
	TransformBroadcaster(ros::NodeHandle &nh);

	// virtual destructor
	~TransformBroadcaster(){ROS_INFO("Destructor called!");}

	// methods
	void odometryCallback(const nav_msgs::Odometry &msg);
	void tf2Broadcaster();
	void spin();

private:

	// methods
	bool readParameters();

	// private nodehandle
	ros::NodeHandle &nh_private;

	// params
	int run_frequency;
	int message_buffer;

	// message type
	nav_msgs::Odometry odom;

	// subscriber object
	std::string subscriber_topic;
	ros::Subscriber robot_state_sub;

	// publisher object
	tf2_ros::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped transformStamped;

};

}