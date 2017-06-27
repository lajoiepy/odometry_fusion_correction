#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"

nav_msgs::Odometry odom1;
nav_msgs::Odometry odom2;
bool first_odom1_received = false;
bool first_odom2_received = false;

void odometry1_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	odom1 = *odom_msg;
	if(!first_odom1_received) first_odom1_received = true;
}

void odometry1_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	odom2 = *odom_msg;
	if(!first_odom2_received) first_odom2_received = true;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "odometry_fusion_correction");

	static ros::NodeHandle n;

	std::string odom_topic1 = "/odometry";
	std::string odom_topic2 = "/rtabmap/odom";

	ros::Subscriber sub_1 = n.subscribe(odom_topic1, 1, odometry1_callback);
	ros::Subscriber sub_2 = n.subscribe(odom_topic2, 1, odometry2_callback);

	ros::Publisher pub = n.advertise<nav_msgs::Odometry>(odom_topic1+"/corrected", 1);

	bool initialization_done = false;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		nav_msgs::Odometry odom_corrected;
		geometry_msgs::Point position_correction;
		geometry_msgs::Quaternion orientation_correction;

		if (!initialization_done && first_odom1_received && first_odom2_received)
		{
			position_correction = odom2.pose.pose.position - odom1.pose.pose.position;
			
			tf::Quaternion odom1_orientation;
			tf::quaternionMsgToTF(odom1.pose.pose.orientation, odom1_orientation);
			tf::Quaternion odom2_orientation;
			tf::quaternionMsgToTF(odom2.pose.pose.orientation, odom2_orientation);

			// Difference between quaternion.

			orientation_correction;
		}
		
		if (initialization_done)
		{
			// Correction
			pub.publish(odom_corrected);
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}