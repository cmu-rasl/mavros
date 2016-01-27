/**
 * @brief servo output raw plugin
 * @file wrench_command.cpp
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

#include <mavros_msgs/WrenchCommand.h>

namespace mavplugin {

  /**
   * @brief servo output raw plugin
   *
   * This plugin allows publishing normalized torque and thrust commands sent by the attitude controller into the mixer
   */
  class WrenchCommandPlugin : public MavRosPlugin {
  public:
    WrenchCommandPlugin() :
      wrc_nh_("~wrench_command"),
      uas_(nullptr)
    { };

    void initialize(UAS &uas)
    {
      uas_ = &uas;
		  wrc_nh_.param<std::string>("frame_id", frame_id, "");
      wrc_pub_ = wrc_nh_.advertise<mavros_msgs::WrenchCommand>("normalized_wrench", 10);
    }

    const message_map get_rx_handlers() {
      return {
        MESSAGE_HANDLER(MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET, &WrenchCommandPlugin::handle_wrench_command)
      };
    }

  private:
    ros::NodeHandle wrc_nh_;
    UAS *uas_;
    ros::Publisher wrc_pub_;

    std::string frame_id;

    void handle_wrench_command(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
      mavlink_actuator_control_target_t wrc_mavlink_msg;
      mavlink_msg_actuator_control_target_decode(msg, &wrc_mavlink_msg);

      // only publish data from control group 0 (attitude control)
      if (wrc_mavlink_msg.group_mlx == 0)
      {
        auto wrc_msg = boost::make_shared<mavros_msgs::WrenchCommand>();

        auto header = uas_->synchronized_header(frame_id, wrc_mavlink_msg.time_usec);

        wrc_msg->header = header;

        int num_controls = sizeof(wrc_mavlink_msg.controls) / sizeof(wrc_mavlink_msg.controls[0]);

        for (int i = 0; i < num_controls; i++)
          wrc_msg->data[i] = wrc_mavlink_msg.controls[i];

        wrc_pub_.publish(wrc_msg);
      }
    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::WrenchCommandPlugin, mavplugin::MavRosPlugin)
