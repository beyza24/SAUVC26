#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class AbduRovPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;
    base_link_ = model_->GetLink(link_name);
    if (!base_link_ && !model_->GetLinks().empty()) {
      base_link_ = model_->GetLinks().front();
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "ABDU ROV: '%s' yok, '%s' kullaniliyor",
        link_name.c_str(), base_link_->GetName().c_str());
    }
    if (!base_link_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "ABDU ROV: hic link yok");
      return;
    }

    water_level_ = sdf->Get<double>("water_level", 0.0).first;
    fluid_density_ = sdf->Get<double>("fluid_density", 1025.0).first;
    gravity_ = std::abs(world_->Gravity().Z());
    displaced_volume_ = sdf->Get<double>("displaced_volume", 0.0162).first;
    max_surge_ = sdf->Get<double>("max_surge_force", 90.0).first;
    max_sway_ = sdf->Get<double>("max_sway_force", 70.0).first;
    max_heave_ = sdf->Get<double>("max_heave_force", 80.0).first;
    max_roll_t_ = sdf->Get<double>("max_roll_torque", 8.0).first;
    max_pitch_t_ = sdf->Get<double>("max_pitch_torque", 10.0).first;
    max_yaw_t_ = sdf->Get<double>("max_yaw_torque", 12.0).first;
    max_speed_ = sdf->Get<double>("max_speed", 1.4).first;
    max_heave_speed_ = sdf->Get<double>("max_heave_speed", 0.55).first;
    max_yaw_rate_ = sdf->Get<double>("max_yaw_rate", 1.1).first;
    cmd_timeout_ = sdf->Get<double>("cmd_timeout", 0.8).first;
    publish_tf_ = sdf->Get<bool>("publish_tf", true).first;
    odom_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_link").first;
    restore_roll_ = sdf->Get<double>("restore_roll", 28.0).first;
    restore_pitch_ = sdf->Get<double>("restore_pitch", 32.0).first;
    current_x_ = sdf->Get<double>("current_x", 0.04).first;
    current_y_ = sdf->Get<double>("current_y", -0.02).first;

    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&AbduRovPlugin::OnCmdVel, this, std::placeholders::_1));
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    depth_pub_ = ros_node_->create_publisher<std_msgs::msg::Float32>("depth", rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);
    }

    last_cmd_time_ = world_->SimTime();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AbduRovPlugin::OnUpdate, this));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "ABDU ROV eklentisi yuklendi (hacim %.4f m3, su seviyesi %.2f m)",
      displaced_volume_, water_level_);
  }

private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_ = *msg;
    last_cmd_time_ = world_->SimTime();
  }

  static double Clamp(double value, double lo, double hi)
  {
    return std::max(lo, std::min(hi, value));
  }

  double SubmergedFraction() const
  {
    const double z = base_link_->WorldPose().Pos().Z();
    const double half = 0.13;
    const double top = z + half;
    const double bottom = z - half;
    if (top <= water_level_) {
      return 1.0;
    }
    if (bottom >= water_level_) {
      return 0.0;
    }
    return Clamp((water_level_ - bottom) / (2.0 * half), 0.0, 1.0);
  }

  void ApplyBuoyancy()
  {
    const double frac = SubmergedFraction();
    if (frac <= 1e-6) {
      return;
    }
    const double force_z = displaced_volume_ * fluid_density_ * gravity_ * frac;
    base_link_->AddForce(ignition::math::Vector3d(0.0, 0.0, force_z));

    const auto rot = base_link_->WorldPose().Rot();
    const double roll = rot.Roll();
    const double pitch = rot.Pitch();
    const ignition::math::Vector3d restore(
      -restore_roll_ * roll,
      -restore_pitch_ * pitch,
      0.0);
    base_link_->AddRelativeTorque(restore * frac);
  }

  void ApplyHydrodynamics(
    const ignition::math::Vector3d & lin, const ignition::math::Vector3d & ang)
  {
    const double frac = std::max(0.15, SubmergedFraction());
    const double Xu = 18.0, Xuu = 42.0;
    const double Yv = 28.0, Yvv = 58.0;
    const double Zw = 36.0, Zww = 72.0;
    const double Kp = 4.5, Kpp = 8.0;
    const double Mq = 5.5, Mqq = 10.0;
    const double Nr = 4.0, Nrr = 7.5;

    ignition::math::Vector3d force(
      (-Xu * lin.X() - Xuu * std::abs(lin.X()) * lin.X()) * frac,
      (-Yv * lin.Y() - Yvv * std::abs(lin.Y()) * lin.Y()) * frac,
      (-Zw * lin.Z() - Zww * std::abs(lin.Z()) * lin.Z()) * frac);
    ignition::math::Vector3d torque(
      (-Kp * ang.X() - Kpp * std::abs(ang.X()) * ang.X()) * frac,
      (-Mq * ang.Y() - Mqq * std::abs(ang.Y()) * ang.Y()) * frac,
      (-Nr * ang.Z() - Nrr * std::abs(ang.Z()) * ang.Z()) * frac);
    base_link_->AddRelativeForce(force);
    base_link_->AddRelativeTorque(torque);

    if (frac > 0.4) {
      const ignition::math::Vector3d current_world(current_x_, current_y_, 0.0);
      const ignition::math::Vector3d current_body =
        base_link_->WorldPose().Rot().RotateVectorReverse(current_world);
      base_link_->AddRelativeForce(ignition::math::Vector3d(
        12.0 * current_body.X(),
        16.0 * current_body.Y(),
        0.0));
    }
  }

  void ApplyThrusters(const geometry_msgs::msg::Twist & cmd, const common::Time & now)
  {
    const double age = (now - last_cmd_time_).Double();
    if (age > cmd_timeout_) {
      return;
    }
    const double frac = SubmergedFraction();
    const double surge_n = Clamp(cmd.linear.x / max_speed_, -1.0, 1.0);
    const double sway_n = Clamp(cmd.linear.y / max_speed_, -1.0, 1.0);
    const double heave_n = Clamp(cmd.linear.z / max_heave_speed_, -1.0, 1.0);
    const double roll_n = Clamp(cmd.angular.x, -1.0, 1.0);
    const double pitch_n = Clamp(cmd.angular.y, -1.0, 1.0);
    const double yaw_n = Clamp(cmd.angular.z / max_yaw_rate_, -1.0, 1.0);

    ignition::math::Vector3d force(
      surge_n * max_surge_ * frac,
      sway_n * max_sway_ * frac,
      heave_n * max_heave_ * frac);
    ignition::math::Vector3d torque(
      roll_n * max_roll_t_ * frac,
      pitch_n * max_pitch_t_ * frac,
      yaw_n * max_yaw_t_ * frac);

    const double z = base_link_->WorldPose().Pos().Z();
    if (z > water_level_ - 0.08 && force.Z() > 0.0) {
      force.Z(0.0);
    }
    base_link_->AddRelativeForce(force);
    base_link_->AddRelativeTorque(torque);
  }

  void PublishState(const common::Time & now)
  {
    const auto pose = base_link_->WorldPose();
    const auto lin = base_link_->RelativeLinearVel();
    const auto ang = base_link_->RelativeAngularVel();
    const auto q = pose.Rot();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = now.sec;
    odom.header.stamp.nanosec = now.nsec;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = q.X();
    odom.pose.pose.orientation.y = q.Y();
    odom.pose.pose.orientation.z = q.Z();
    odom.pose.pose.orientation.w = q.W();
    odom.twist.twist.linear.x = lin.X();
    odom.twist.twist.linear.y = lin.Y();
    odom.twist.twist.linear.z = lin.Z();
    odom.twist.twist.angular.x = ang.X();
    odom.twist.twist.angular.y = ang.Y();
    odom.twist.twist.angular.z = ang.Z();
    odom_pub_->publish(odom);

    std_msgs::msg::Float32 depth;
    depth.data = static_cast<float>(water_level_ - pose.Pos().Z());
    depth_pub_->publish(depth);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pose.Pos().X();
      tf.transform.translation.y = pose.Pos().Y();
      tf.transform.translation.z = pose.Pos().Z();
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void OnUpdate()
  {
    if (!base_link_) {
      return;
    }
    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd = cmd_;
    }
    const auto now = world_->SimTime();
    ApplyBuoyancy();
    ApplyHydrodynamics(base_link_->RelativeLinearVel(), base_link_->RelativeAngularVel());
    ApplyThrusters(cmd, now);
    PublishState(now);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr base_link_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  event::ConnectionPtr update_connection_;
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist cmd_;
  common::Time last_cmd_time_;
  double water_level_{0.0};
  double fluid_density_{1025.0};
  double gravity_{9.81};
  double displaced_volume_{0.0162};
  double max_surge_{90.0};
  double max_sway_{70.0};
  double max_heave_{80.0};
  double max_roll_t_{8.0};
  double max_pitch_t_{10.0};
  double max_yaw_t_{12.0};
  double max_speed_{1.4};
  double max_heave_speed_{0.55};
  double max_yaw_rate_{1.1};
  double cmd_timeout_{0.8};
  double restore_roll_{28.0};
  double restore_pitch_{32.0};
  double current_x_{0.04};
  double current_y_{-0.02};
  bool publish_tf_{true};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
};

GZ_REGISTER_MODEL_PLUGIN(AbduRovPlugin)
}  // namespace gazebo
