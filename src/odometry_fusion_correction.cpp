#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
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

void odometry2_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
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

	ros::Publisher pub = n.advertise<nav_msgs::Odometry>(odom_topic2+"/corrected", 1);

	bool initialization_done = false;

	ros::Rate loop_rate(10);

	geometry_msgs::Point position_correction;
	tf::Quaternion orientation_correction;

	tf::TransformBroadcaster br;
	tf::Transform transform_correction;

	while (ros::ok())
	{
		nav_msgs::Odometry odom_corrected;

		if (!initialization_done && first_odom1_received && first_odom2_received)
		{			
			tf::Quaternion odom1_orientation;
			tf::quaternionMsgToTF(odom1.pose.pose.orientation, odom1_orientation);
			
			transform_correction = tf::Transform(odom1_orientation, tf::Vector3(odom1.pose.pose.position.x,odom1.pose.pose.position.y,odom1.pose.pose.position.z));
			
			initialization_done = true;
		}
		
		if (initialization_done)
		{
			tf::Quaternion odom2_orientation;
			tf::quaternionMsgToTF(odom2.pose.pose.orientation, odom2_orientation);
			
			tf::Transform odom2_transform(odom2_orientation, tf::Vector3(odom2.pose.pose.position.x,odom2.pose.pose.position.y,odom2.pose.pose.position.z));
			
			odom2_transform = transform_correction * odom2_transform;

			// Position correction.
			odom_corrected = odom2;
			odom_corrected.pose.pose.position.x = odom2_transform.getOrigin().getX();
			odom_corrected.pose.pose.position.y = odom2_transform.getOrigin().getY();
			odom_corrected.pose.pose.position.z = odom2_transform.getOrigin().getZ();
			tf::quaternionTFToMsg(odom2_transform.getRotation(), odom_corrected.pose.pose.orientation);
			
			// Corrected odometry publication.
			pub.publish(odom_corrected);
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}