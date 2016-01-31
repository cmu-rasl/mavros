/**
 * @brief LocalPositionWithCovariance plugin
 * @file local_position_with_covariance.cpp
 * @author John Yao <johnyao@cmu.edu>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 John Yao
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace mavplugin {
/**
 * @brief Local position with covariance plugin.
 * Publish local position to TF and Odometry
 */
class LocalPositionWithCovariancePlugin : public MavRosPlugin {
public:
	LocalPositionWithCovariancePlugin() :
		lp_nh("~local_position_with_covariance"),
		uas(nullptr),
		tf_send(false)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		lp_nh.param<std::string>("frame_id", frame_id, "fcu");
		// tf subsection
		lp_nh.param("tf/send", tf_send, true);
		lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "local_origin");
		lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "fcu");

		lpwc_pub = lp_nh.advertise<nav_msgs::Odometry>("state", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV, &LocalPositionWithCovariancePlugin::handle_local_position_ned_cov)
		};
	}

private:
	ros::NodeHandle lp_nh;
	UAS *uas;

	ros::Publisher lpwc_pub;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;

	void handle_local_position_ned_cov(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_cov_t pnc;
		mavlink_msg_local_position_ned_cov_decode(msg, &pnc);

		auto position = UAS::transform_frame_ned_enu(Eigen::Vector3d(pnc.x, pnc.y, pnc.z));
		auto velocity = UAS::transform_frame_ned_enu(Eigen::Vector3d(pnc.vx, pnc.vy, pnc.vz));

    // note that the ax, ay, and az fields are body acceleration bias and NOT acceleration
    auto accel_bias = UAS::transform_frame_ned_enu(Eigen::Vector3d(pnc.ax, pnc.ay, pnc.az));

		auto orientation = uas->get_attitude_orientation();
		auto angular_velocity = uas->get_attitude_angular_velocity();

		auto odom = boost::make_shared<nav_msgs::Odometry>();

		odom->header = uas->synchronized_header(frame_id, pnc.time_boot_ms);

		tf::pointEigenToMsg(position, odom->pose.pose.position);
		odom->pose.pose.orientation = orientation;

		tf::vectorEigenToMsg(velocity,odom->twist.twist.linear);
		odom->twist.twist.angular = angular_velocity;

    for (unsigned int i = 0; i < 3; i++)
    {
      odom->pose.covariance[i*7] = pnc.covariance[i];
      odom->twist.covariance[i*7] = pnc.covariance[i+3];

      // HACK: stuff acceleration bias and its corresponding covariance diagonals into
      // unused fields of the twist covariance
      odom->twist.covariance[21 + i] = accel_bias(i);
      odom->twist.covariance[27 + i] = pnc.covariance[i+6];
    }

    // HACK: stuff mocap position observation into unused fields of the twist covariance
    // manually do NED to NWU transform
    odom->twist.covariance[33] = pnc.covariance[9];
    odom->twist.covariance[34] = -pnc.covariance[10];
    odom->twist.covariance[35] = -pnc.covariance[11];

   	lpwc_pub.publish(odom);

		if (tf_send) {
			geometry_msgs::TransformStamped transform;

			transform.header.stamp = odom->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;

			transform.transform.rotation = orientation;
			tf::vectorEigenToMsg(position, transform.transform.translation);

			uas->tf2_broadcaster.sendTransform(transform);
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionWithCovariancePlugin, mavplugin::MavRosPlugin)
