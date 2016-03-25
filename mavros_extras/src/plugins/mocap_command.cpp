#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <quadrotor_msgs/CascadedCommand.h>
#include <quadrotor_msgs/CascadedCommandGains.h>
#include <std_msgs/Bool.h>
#include <quadrotor_srvs/Toggle.h>

namespace mavplugin {
  /**
   * @brief MocapCommand plugin
   *
   * Sends commands to be used during motion capture flight.
   */
  class MocapCommandPlugin : public MavRosPlugin
  {
  public:
    MocapCommandPlugin() :
      nh("~mocap_command"),
      uas(nullptr) { }

    void initialize(UAS &uas_)
    {
      uas = &uas_;

      cascaded_cmd_sub =
        nh.subscribe("cascaded_cmd", 1,
                     &MocapCommandPlugin::cascadedCommandMessageCallback, this);

      cascaded_cmd_gains_sub =
        nh.subscribe("cascaded_cmd_gains", 1,
                     &MocapCommandPlugin::cascadedCommandGainsMessageCallback, this);

      motors_sub =
        nh.subscribe("motors", 1, &MocapCommandPlugin::motorMessageCallback, this);

      motors_service =
        nh.advertiseService("motors", &MocapCommandPlugin::motorServiceCallback, this);
    }

    const message_map get_rx_handlers()
    {
      return { /* Rx disabled */ };
    }

  private:
    ros::NodeHandle nh;
    UAS *uas;

    ros::Subscriber cascaded_cmd_sub;
    ros::Subscriber cascaded_cmd_gains_sub;
    ros::Subscriber motors_sub;
    ros::ServiceServer motors_service;

    void sendCascadedCommand(const quadrotor_msgs::CascadedCommand& msg)
    {
      Eigen::Vector3d thrust =
        UAS::transform_frame_enu_ned(Eigen::Vector3d(0.0, 0.0, msg.thrust));

      Eigen::Quaterniond q_enu;
      float q[4];

      tf::quaternionMsgToEigen(msg.orientation, q_enu);
      UAS::quaternion_to_mavlink(UAS::transform_frame_enu_ned(q_enu), q);

      Eigen::Vector3d ang_vel =
        UAS::transform_frame_enu_ned(Eigen::Vector3d(msg.angular_velocity.x,
                                                     msg.angular_velocity.y,
                                                     msg.angular_velocity.z));

      Eigen::Vector3d ang_acc =
        UAS::transform_frame_enu_ned(Eigen::Vector3d(msg.angular_acceleration.x,
                                                     msg.angular_acceleration.y,
                                                     msg.angular_acceleration.z));

      mavlink_cascaded_cmd_t cmd;
      cmd.target_system = uas->get_tgt_system();
      cmd.thrust = thrust(2);
      for (unsigned int i = 0; i < 4; i++)
        cmd.q[i] = q[i];
      for (unsigned int i = 0; i < 3; i++)
        cmd.ang_vel[i] = ang_vel[i];
      for (unsigned int i = 0; i < 3; i++)
        cmd.ang_acc[i] = ang_acc[i];

      mavlink_message_t mmsg;
      mavlink_msg_cascaded_cmd_encode(0, 0, &mmsg, &cmd);
      UAS_FCU(uas)->send_message(&mmsg);
    }

    void sendCascadedCommandGains(const quadrotor_msgs::CascadedCommandGains& msg)
    {
      mavlink_cascaded_cmd_gains_t cmd;
      cmd.target_system = uas->get_tgt_system();
      cmd.kR[0] = msg.kR.x;
      cmd.kR[1] = msg.kR.y;
      cmd.kR[2] = msg.kR.z;

      cmd.kOm[0] = msg.kOm.x;
      cmd.kOm[1] = msg.kOm.y;
      cmd.kOm[2] = msg.kOm.z;

      mavlink_message_t mmsg;
      mavlink_msg_cascaded_cmd_gains_encode(0, 0, &mmsg, &cmd);
      UAS_FCU(uas)->send_message(&mmsg);
    }

    void sendMocapMotorState(bool enable_motors)
    {
      mavlink_mocap_motor_state_t cmd;
      cmd.target_system = uas->get_tgt_system();
      cmd.state = enable_motors ? 1 : 0;

      mavlink_message_t mmsg;
      mavlink_msg_mocap_motor_state_encode(0, 0, &mmsg, &cmd);
      UAS_FCU(uas)->send_message(&mmsg);
    }

    void cascadedCommandMessageCallback(const quadrotor_msgs::CascadedCommand::ConstPtr& msg)
    {
      sendCascadedCommand(*msg);
    }

    void cascadedCommandGainsMessageCallback(const quadrotor_msgs::CascadedCommandGains::ConstPtr& msg)
    {
      sendCascadedCommandGains(*msg);
    }

    void motorMessageCallback(const std_msgs::Bool::ConstPtr& msg)
    {
      sendMocapMotorState(msg->data);
    }

    bool motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq,
                              quadrotor_srvs::Toggle::Response& mres)
    {
      mres.status = mreq.enable;
      sendMocapMotorState(mreq.enable);
      return true;
    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapCommandPlugin, mavplugin::MavRosPlugin)
