/**
 * @brief MocapPoseEstimate plugin
 * @file mocap_pose_estimate.cpp
 * @author Tony Baltovski <tony.baltovski@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov, Tony Baltovski.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <thread>
#include <condition_variable>
namespace mavplugin {
/**
 * @brief MocapPoseEstimate plugin
 *
 * Sends motion capture data to FCU.
 */
class MocapPoseEstimatePlugin : public MavRosPlugin
{
public:
	MocapPoseEstimatePlugin() :
		is_init_(false),
		mp_nh("~mocap"),
		uas(nullptr),
		last_time_us_(0.0f),
		last_last_(0.0f),
		rate_(60.0f),
		thread_(&MocapPoseEstimatePlugin::publish_mavlink_message_at_rate, this)
	{ };

	void initialize(UAS &uas_)
	{
		bool use_tf;
		bool use_pose;

		uas = &uas_;

		/** @note For VICON ROS package, subscribe to TransformStamped topic */
		mp_nh.param("use_tf", use_tf, false);

		/** @note For Optitrack ROS package, subscribe to PoseStamped topic */
		mp_nh.param("use_pose", use_pose, true);

		mp_nh.param("rate", rate_, 60.0f);

		if (use_tf && !use_pose) {
			mocap_tf_sub = mp_nh.subscribe("tf", 1, &MocapPoseEstimatePlugin::mocap_tf_cb, this);
			std::cout<<"[Mocap Pose Plugin] Using tf!!!\n";		
}
		else if (use_pose && !use_tf) {
			mocap_pose_sub = mp_nh.subscribe("pose", 1, &MocapPoseEstimatePlugin::mocap_pose_cb, this);
		
			std::cout<<"[Mocap Pose Plugin] Using PoseStamped!!!\n";			
}
		else {
			ROS_ERROR_NAMED("mocap", "Use one motion capture source.");
		}
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle mp_nh;
	UAS *uas;

	ros::Subscriber mocap_pose_sub;
	ros::Subscriber mocap_tf_sub;
	ros::Time last_ros_;
	double last_time_us_, last_last_;
	float  rate_, last_q_[4];
	bool is_init_;
	Eigen::Vector3d last_pos_;
	std::mutex lock_;
	std::condition_variable cv_;
	std::thread thread_;
	void publish_mavlink_message_at_rate(){
		//Wait until we're ready to initialize
		std::cout<<"Waiting for init...\n";
		{
		std::unique_lock<std::mutex> locker(lock_);
		cv_.wait(locker, [&]{return is_init_;});
		}
		auto frame_duration = std::chrono::duration<double>(1.0/rate_);
		do{
			auto start_time = std::chrono::steady_clock::now();
			auto end_time = start_time + frame_duration;
			{
				std::lock_guard<std::mutex> locker(lock_);
				if(last_last_ == 0.0f || last_time_us_ - last_last_ > 1e-10){
					mocap_pose_send(last_time_us_, last_q_, last_pos_[0], last_pos_[1], last_pos_[2]);
					last_last_ = last_time_us_;
				}	
}
			last_ros_ = ros::Time::now();
			std::this_thread::sleep_until(end_time);
		}while(ros::ok());
	}
	/* -*- low-level send -*- */
	void mocap_pose_send
		(uint64_t usec,
			float q[4],
			float x, float y, float z)
	{
		mavlink_message_t msg;
		mavlink_msg_att_pos_mocap_pack_chan(UAS_PACK_CHAN(uas), &msg,
				usec,
				q,
				x,
				y,
				z);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */
	void mocap_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		Eigen::Quaterniond q_enu;
		float q[4];

		tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
		UAS::quaternion_to_mavlink(
				UAS::transform_orientation_enu_ned(
					UAS::transform_orientation_baselink_aircraft(q_enu)),
				q);

		auto position = UAS::transform_frame_enu_ned(
				Eigen::Vector3d(
					pose->pose.position.x,
					pose->pose.position.y,
					pose->pose.position.z));

		mocap_pose_send(pose->header.stamp.toNSec() / 1000,
				q,
				position.x(),
				position.y(),
				position.z());
	}

	/* -*- callbacks -*- */
	void mocap_tf_cb(const geometry_msgs::TransformStamped::ConstPtr &trans)
	{
		Eigen::Quaterniond q_enu;
		float q[4];

		tf::quaternionMsgToEigen(trans->transform.rotation, q_enu);
		UAS::quaternion_to_mavlink(
				UAS::transform_orientation_enu_ned(
					UAS::transform_orientation_baselink_aircraft(q_enu)),
				q);

		auto position = UAS::transform_frame_enu_ned(
				Eigen::Vector3d(
					trans->transform.translation.x,
					trans->transform.translation.y,
					trans->transform.translation.z));
		double time_us = trans->header.stamp.toNSec() / 1000;
		{
			std::lock_guard<std::mutex> locker(lock_);
			last_time_us_ = time_us;
			for(int i=0;i<4;i++){last_q_[i] = q[i];}
			last_pos_ = position;
			//Start thread
			if(!is_init_){
				is_init_ = true;
				cv_.notify_one();
			}
		}
		
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapPoseEstimatePlugin, mavplugin::MavRosPlugin)
