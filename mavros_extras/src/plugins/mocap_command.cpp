#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <quadrotor_msgs/RPMCommand.h>
#include <quadrotor_msgs/CascadedCommand.h>
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
      uas(nullptr),
      enable_motors(false) { };

    void initialize(UAS &uas_)
    {
      uas = &uas_;

      cascaded_cmd_sub =
        nh.subscribe("cascaded_cmd", 1,
                     &MocapCommandPlugin::cascadedCommandMessageCallback, this);

      rpm_cmd_sub =
        nh.subscribe("rpm_cmd", 1,
                     &MocapCommandPlugin::rpmCommandMessageCallback, this);

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

    bool enable_motors;

    ros::Subscriber cascaded_cmd_sub;
    ros::Subscriber rpm_cmd_sub;
    ros::Subscriber motors_sub;
    ros::ServiceServer motors_service;

    void sendRPMCommand(const quadrotor_msgs::RPMCommand& msg)
    {
      // cmd type = 1 (rpm)
      sendMavlinkCommand(msg.motor_rpm[0], msg.motor_rpm[1],
                         msg.motor_rpm[2], msg.motor_rpm[3], 1, 0);
    }

    void sendCascadedCommand(const quadrotor_msgs::CascadedCommand& msg)
    {
      // cmd type = 2 (cascaded)
      // input types:
      // 0: thrust setpoint
      // 1: att (quat) setpoint
      // 2: angular velocities
      // 3: angular accelerations
      // 4: pgains
      // 5: dgains

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

      sendMavlinkCommand(thrust.z(), 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
      sendMavlinkCommand(q[0], q[1], q[2], q[3], 2.0f, 1.0f);
      sendMavlinkCommand(ang_vel.x(), ang_vel.y(), ang_vel.z(), 0.0f, 2.0f, 2.0f);
      sendMavlinkCommand(ang_acc.x(), ang_acc.y(), ang_acc.z(), 0.0f, 2.0f, 3.0f);
      sendMavlinkCommand(msg.kR.x, msg.kR.y, msg.kR.z, 0.0f, 2.0f, 4.0f);
      sendMavlinkCommand(msg.kOm.x, msg.kOm.y, msg.kOm.z, 0.0f, 2.0f, 5.0f);
    }

    void sendMavlinkCommand(float p1, float p2, float p3, float p4, float p5, float p6)
    {
      mavlink_command_long_t cmd;
      cmd.target_system = uas->get_tgt_system();
      cmd.target_component = uas->get_tgt_component();
      cmd.param1 = p1;
      cmd.param2 = p2;
      cmd.param3 = p3;
      cmd.param4 = p4;
      cmd.param5 = p5;
      cmd.param6 = p6;
      cmd.param7 = enable_motors ? 1.0f : 0.0f;

      mavlink_message_t mmsg;
      mavlink_msg_command_long_encode(0, 0, &mmsg, &cmd);
      UAS_FCU(uas)->send_message(&mmsg);
    }

    void rpmCommandMessageCallback(const quadrotor_msgs::RPMCommand::ConstPtr& msg)
    {
      sendRPMCommand(*msg);
    }

    void cascadedCommandMessageCallback(const quadrotor_msgs::CascadedCommand::ConstPtr& msg)
    {
      sendCascadedCommand(*msg);
    }

    void motorMessageCallback(const std_msgs::Bool::ConstPtr& msg)
    {
      enable_motors = msg->data;
    }

    bool motorServiceCallback(quadrotor_srvs::Toggle::Request& mreq,
                              quadrotor_srvs::Toggle::Response& mres)
    {
      mres.status = enable_motors = mreq.enable;
      return true;
    }
  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::MocapCommandPlugin, mavplugin::MavRosPlugin)
