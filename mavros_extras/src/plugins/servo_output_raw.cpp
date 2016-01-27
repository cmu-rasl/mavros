/**
 * @brief servo output raw plugin
 * @file servo_output_raw.cpp
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
//#include <unordered_map>
//#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
//#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/ServoOutputRaw.h>

namespace mavplugin {

  /**
   * @brief servo output raw plugin
   *
   * This plugin allows publishing servo outputs (e.g. PWM)
   */
  class ServoOutputRawPlugin : public MavRosPlugin {
  public:
    ServoOutputRawPlugin() :
      sor_nh_("~servo_output_raw"),
      uas_(nullptr)
    { };

    void initialize(UAS &uas)
    {
      uas_ = &uas;
		  sor_nh_.param<std::string>("frame_id", frame_id, "servo");
      sor_pub_ = sor_nh_.advertise<mavros_msgs::ServoOutputRaw>("servo_output_raw", 10);
    }

    const message_map get_rx_handlers() {
      return {
        MESSAGE_HANDLER(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, &ServoOutputRawPlugin::handle_servo_output_raw)
      };
    }

  private:
    ros::NodeHandle sor_nh_;
    UAS *uas_;
    ros::Publisher sor_pub_;

    std::string frame_id;

    void handle_servo_output_raw(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
      mavlink_servo_output_raw_t sor_mavlink_msg;
      mavlink_msg_servo_output_raw_decode(msg, &sor_mavlink_msg);

      auto sor_msg = boost::make_shared<mavros_msgs::ServoOutputRaw>();

		  auto header = uas_->synchronized_header(frame_id, sor_mavlink_msg.time_usec);

      sor_msg->header = header;

      sor_msg->port = sor_mavlink_msg.port;

      sor_msg->data[0] = sor_mavlink_msg.servo1_raw;
      sor_msg->data[1] = sor_mavlink_msg.servo2_raw;
      sor_msg->data[2] = sor_mavlink_msg.servo3_raw;
      sor_msg->data[3] = sor_mavlink_msg.servo4_raw;
      sor_msg->data[4] = sor_mavlink_msg.servo5_raw;
      sor_msg->data[5] = sor_mavlink_msg.servo6_raw;
      sor_msg->data[6] = sor_mavlink_msg.servo7_raw;
      sor_msg->data[7] = sor_mavlink_msg.servo8_raw;

      sor_pub_.publish(sor_msg);

    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ServoOutputRawPlugin, mavplugin::MavRosPlugin)
