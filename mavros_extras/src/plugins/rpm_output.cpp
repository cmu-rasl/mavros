/**
 * @brief rpm output plugin
 * @file rpm_output.cpp
 * @author John Yao <johnyao@cmu.edu>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 John Yao.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/RPMOutput.h>

namespace mavplugin {

  /**
   * @brief rpm output plugin
   *
   * This plugin allows publishing RPM outputs
   */
  class RPMOutputPlugin : public MavRosPlugin {
  public:
    RPMOutputPlugin() :
      rpo_nh_("~rpm_output"),
      uas_(nullptr)
    { };

    void initialize(UAS &uas)
    {
      uas_ = &uas;
		  rpo_nh_.param<std::string>("frame_id", frame_id, "rpm");
      rpo_pub_ = rpo_nh_.advertise<mavros_msgs::RPMOutput>("data", 10);
    }

    const message_map get_rx_handlers() {
      return {
        MESSAGE_HANDLER(MAVLINK_MSG_ID_RPM_OUTPUT, &RPMOutputPlugin::handle_rpm_output)
      };
    }

  private:
    ros::NodeHandle rpo_nh_;
    UAS *uas_;
    ros::Publisher rpo_pub_;

    std::string frame_id;

    void handle_rpm_output(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
      mavlink_rpm_output_t rpo_mavlink_msg;
      mavlink_msg_rpm_output_decode(msg, &rpo_mavlink_msg);

      auto rpo_msg = boost::make_shared<mavros_msgs::RPMOutput>();

		  auto header = uas_->synchronized_header(frame_id, rpo_mavlink_msg.time_usec);

      rpo_msg->header = header;

      // PWM (and hence RPM) outputs are stored at indices 4 to 7
      for (int i = 0; i < 4; i++)
        rpo_msg->data.push_back(rpo_mavlink_msg.output[i+4]);

      rpo_pub_.publish(rpo_msg);
    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::RPMOutputPlugin, mavplugin::MavRosPlugin)
