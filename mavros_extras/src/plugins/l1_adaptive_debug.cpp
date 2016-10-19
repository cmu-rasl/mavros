/**
 * @brief l1 Adaptive Debug plugin
 * @file l1_adaptive_debug.cpp
 * @author Kumar Shaurya Shankar <kumarsha@cs.cmu.edu>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Kumar Shaurya Shankar.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <unordered_map>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/L1AdaptiveDebug.h>

namespace mavplugin {

  /**
   * @brief L1 Adaptive Debug plugin
   *
   * This plugin allows publishing debug information sent down by the L1 attitude estimator
   */
  class L1AdaptiveDebugPlugin : public MavRosPlugin {
  public:
    L1AdaptiveDebugPlugin() :
      l1ac_nh_("~l1_adaptive_debug"),
      uas_(nullptr)
    { };

    void initialize(UAS &uas)
    {
      uas_ = &uas;
      l1ac_pub_ = l1ac_nh_.advertise<mavros_msgs::L1AdaptiveDebug>("l1ac_debug", 10);
    }

    const message_map get_rx_handlers() {
      return {
        MESSAGE_HANDLER(MAVLINK_MSG_ID_L1_ADAPTIVE_DEBUG, &L1AdaptiveDebugPlugin::handle_l1_adaptive_debug)
      };
    }

  private:
    ros::NodeHandle l1ac_nh_;
    UAS *uas_;
    ros::Publisher l1ac_pub_;

    void handle_l1_adaptive_debug(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
      mavlink_l1_adaptive_debug_t l1ac_mavlink_msg;
      mavlink_msg_l1_adaptive_debug_decode(msg, &l1ac_mavlink_msg);

      auto l1ac_msg = boost::make_shared<mavros_msgs::L1AdaptiveDebug>();

      l1ac_msg->header.stamp = ros::Time(l1ac_mavlink_msg.timestamp);

      l1ac_msg->avl.x = l1ac_mavlink_msg.avl_hat[0];
      l1ac_msg->avl.y = l1ac_mavlink_msg.avl_hat[1];
      l1ac_msg->avl.z = l1ac_mavlink_msg.avl_hat[2];

      l1ac_msg->dst.x = l1ac_mavlink_msg.dst_hat[0];
      l1ac_msg->dst.y = l1ac_mavlink_msg.dst_hat[1];
      l1ac_msg->dst.z = l1ac_mavlink_msg.dst_hat[2];

      l1ac_msg->euler_rates.x = l1ac_mavlink_msg.ang_vel[0];
      l1ac_msg->euler_rates.y = l1ac_mavlink_msg.ang_vel[1];
      l1ac_msg->euler_rates.z = l1ac_mavlink_msg.ang_vel[2];

      l1ac_msg->lpd.x = l1ac_mavlink_msg.lpd[0];
      l1ac_msg->lpd.y = l1ac_mavlink_msg.lpd[1];
      l1ac_msg->lpd.z = l1ac_mavlink_msg.lpd[2];

      l1ac_msg->gyro.x = l1ac_mavlink_msg.rates[0];
      l1ac_msg->gyro.y = l1ac_mavlink_msg.rates[1];
      l1ac_msg->gyro.z = l1ac_mavlink_msg.rates[2];

      l1ac_pub_.publish(l1ac_msg);

    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::L1AdaptiveDebugPlugin, mavplugin::MavRosPlugin)
