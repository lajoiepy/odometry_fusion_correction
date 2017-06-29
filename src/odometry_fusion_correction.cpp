#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

nav_msgs::Odometry odom0;
nav_msgs::Odometry odom1;
bool first_odom0_received = false;
bool first_odom1_received = false;

void odometry0_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	odom0 = *odom_msg;
	if(!first_odom0_received) first_odom0_received = true;
}

void odometry1_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	odom1 = *odom_msg;
	if(!first_odom1_received) first_odom1_received = true;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "odometry_fusion_correction");

	ros::NodeHandle n("~");

	std::string odom0_topic = "/odom0";
	n.getParam("odom0_topic", odom0_topic);
	std::string odom1_topic = "/odom1";
	n.getParam("odom1_topic", odom1_topic);

	ros::Subscriber sub_0 = n.subscribe(odom0_topic, 1, odometry0_callback);
	ros::Subscriber sub_1 = n.subscribe(odom1_topic, 1, odometry1_callback);

	std::string correction_suffix = "/corrected";
	n.getParam("correction_suffix", correction_suffix);
	ros::Publisher pub_0 = n.advertise<nav_msgs::Odometry>(odom0_topic + correction_suffix, 1);
	ros::Publisher pub_1 = n.advertise<nav_msgs::Odometry>(odom1_topic + correction_suffix, 1);

	std::string odom_frame_id = "odom";
	n.getParam("odom_frame_id", odom_frame_id);
	std::string child_frame_id = "base_link";
	n.getParam("child_frame_id", child_frame_id);

	float odom0_covariance_diag_value = -1;
	n.getParam("odom0_covariance_diag_value", odom0_covariance_diag_value);

	float odom1_covariance_diag_value = -1;
	n.getParam("odom1_covariance_diag_value", odom1_covariance_diag_value);

	bool initialization_done = false;

	ros::Rate loop_rate(10);

	geometry_msgs::Point position_correction;
	tf::Quaternion orientation_correction;

	tf::TransformBroadcaster br;
	tf::Transform transform_correction;

	while (ros::ok())
	{
		nav_msgs::Odometry odom_corrected;

		if (!initialization_done && first_odom0_received && first_odom1_received)
		{			
			tf::Quaternion odom0_orientation;
			tf::quaternionMsgToTF(odom0.pose.pose.orientation, odom0_orientation);
			
			transform_correction = tf::Transform(odom0_orientation, tf::Vector3(odom0.pose.pose.position.x,odom0.pose.pose.position.y,odom0.pose.pose.position.z));
			
			initialization_done = true;
		}
		
		if (initialization_done)
		{
			tf::Quaternion odom1_orientation;
			tf::quaternionMsgToTF(odom1.pose.pose.orientation, odom1_orientation);
			
			tf::Transform odom1_transform(odom1_orientation, tf::Vector3(odom1.pose.pose.position.x,odom1.pose.pose.position.y,odom1.pose.pose.position.z));
			
			odom1_transform = transform_correction * odom1_transform;

			// Position correction.
			odom_corrected = odom1;
			odom_corrected.pose.pose.position.x = odom1_transform.getOrigin().getX();
			odom_corrected.pose.pose.position.y = odom1_transform.getOrigin().getY();
			odom_corrected.pose.pose.position.z = odom1_transform.getOrigin().getZ();
			tf::quaternionTFToMsg(odom1_transform.getRotation(), odom_corrected.pose.pose.orientation);
			
			// Corrected odometries publication.
			odom_corrected.header.frame_id = odom_frame_id;
			odom_corrected.child_frame_id = child_frame_id;
			if ( odom1_covariance_diag_value != -1 )
				odom_corrected.pose.covariance = { odom1_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom1_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom1_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom1_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom1_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom1_covariance_diag_value };
			pub_1.publish(odom_corrected);

			odom0.header.frame_id = odom_frame_id;
			odom0.child_frame_id = child_frame_id;
			if ( odom0_covariance_diag_value != -1 )
				odom0.pose.covariance = { odom0_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom0_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom0_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom0_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom0_covariance_diag_value, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, odom0_covariance_diag_value };
			pub_0.publish(odom0);
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}