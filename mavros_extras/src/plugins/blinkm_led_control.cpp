#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/UInt8.h>

namespace mavplugin {
  /**
   * @brief BlinkMLEDPlugin plugin
   *
   * Sends LED control input to FCU
   */
  class BlinkMLEDPlugin : public MavRosPlugin
  {
  public:
    BlinkMLEDPlugin() :
      nh("~led_control"),
      uas(nullptr) { }

    void initialize(UAS &uas_)
    {
      uas = &uas_;

      led_control_sub =
        nh.subscribe("led_control", 1,
                     &BlinkMLEDPlugin::ledCommandMessageCallback, this);
    }

    const message_map get_rx_handlers()
    {
      return { /* Rx disabled */ };
    }

  private:
    ros::NodeHandle nh;
    UAS *uas;

    ros::Subscriber led_control_sub;

    void sendLEDCommand(const unsigned char in)
    {
      mavlink_blinkm_control_t cmd;
      cmd.target_system = uas->get_tgt_system();
      cmd.control = in;

      mavlink_message_t mmsg;
      mavlink_msg_blinkm_control_encode(0, 0, &mmsg, &cmd);
      UAS_FCU(uas)->send_message(&mmsg);
    }

    void ledCommandMessageCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
      sendLEDCommand(msg->data);
    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::BlinkMLEDPlugin, mavplugin::MavRosPlugin)
