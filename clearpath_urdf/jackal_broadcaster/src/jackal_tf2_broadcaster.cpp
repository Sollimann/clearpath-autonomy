#include "jackal_broadcaster/jackal_broadcaster_node.hpp"

namespace jackal_tf2_broadcaster_ns {

TransformBroadcaster::TransformBroadcaster(ros::NodeHandle &nh) : nh_private(nh) , run_frequency(100), message_buffer(500)
{
	if (!readParameters())
	{
		ROS_WARN("Failed to read parameter subscriber_topic");
		subscriber_topic = "/jackal_velocity_controller/odom";
		//ros::requestShutdown();
	}

	ROS_INFO("Initialized at %i Hz. \n", run_frequency);

	// subscribers
	robot_state_sub = nh_private.subscribe(subscriber_topic, message_buffer, &TransformBroadcaster::odometryCallback, this);

	ROS_INFO("Successfully launched spot tf2 broadcaster.");

}

bool TransformBroadcaster::readParameters()
{
	if (!nh_private.getParam("subscriber_topic", subscriber_topic)) return false;
	return true;
}


void TransformBroadcaster::spin()
{
	ros::Rate rate(run_frequency);
	while(ros::ok())
	{

		tf2Broadcaster();

		ros::spinOnce();
		rate.sleep();
	}

}


void TransformBroadcaster::odometryCallback(const nav_msgs::Odometry &msg)
{
	odom = msg;
}


void TransformBroadcaster::tf2Broadcaster()
{
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "odom"; //associated inertial frame
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.x = odom.pose.pose.position.x;
	transformStamped.transform.translation.y = odom.pose.pose.position.y;
	transformStamped.transform.translation.z = odom.pose.pose.position.z;
	transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
	transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
	transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
	transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

	broadcaster.sendTransform(transformStamped);
}

}