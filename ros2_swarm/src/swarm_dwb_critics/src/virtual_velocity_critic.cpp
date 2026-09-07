#include "swarm_dwb_critics/virtual_velocity_critic.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace swarm_dwb_critics
{

VelocityTarget make_velocity_target(
  const double reference_body_x,
  const double reference_body_y,
  const double reference_yaw,
  const double reference_yaw_rate,
  const double robot_yaw,
  const nav_2d_msgs::msg::Twist2D & measured_velocity,
  const double linear_feedback_gain,
  const double angular_feedback_gain)
{
  const double reference_cosine = std::cos(reference_yaw);
  const double reference_sine = std::sin(reference_yaw);
  const double world_x =
    reference_cosine * reference_body_x - reference_sine * reference_body_y;
  const double world_y =
    reference_sine * reference_body_x + reference_cosine * reference_body_y;

  const double robot_cosine = std::cos(robot_yaw);
  const double robot_sine = std::sin(robot_yaw);
  const double robot_body_x = robot_cosine * world_x + robot_sine * world_y;
  const double robot_body_y = -robot_sine * world_x + robot_cosine * world_y;

  return {
    robot_body_x + linear_feedback_gain * (robot_body_x - measured_velocity.x),
    robot_body_y + linear_feedback_gain * (robot_body_y - measured_velocity.y),
    reference_yaw_rate +
      angular_feedback_gain * (reference_yaw_rate - measured_velocity.theta)
  };
}

double velocity_target_score(
  const nav_2d_msgs::msg::Twist2D & candidate,
  const VelocityTarget & target,
  const double linear_x_weight,
  const double linear_y_weight,
  const double angular_weight)
{
  const double error_x = candidate.x - target.x;
  const double error_y = candidate.y - target.y;
  const double error_theta = candidate.theta - target.theta;
  return linear_x_weight * error_x * error_x +
         linear_y_weight * error_y * error_y +
         angular_weight * error_theta * error_theta;
}

std::string normalized_frame(const std::string & frame)
{
  const auto first_character = frame.find_first_not_of('/');
  return first_character == std::string::npos ? "" : frame.substr(first_character);
}

bool velocity_reference_is_valid(
  const double age,
  const double timeout,
  const std::string & reference_frame,
  const std::string & costmap_frame)
{
  const std::string normalized_reference = normalized_frame(reference_frame);
  return std::isfinite(age) && age >= 0.0 && age <= timeout &&
         !normalized_reference.empty() &&
         normalized_reference == normalized_frame(costmap_frame);
}

void VirtualVelocityCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock controller node");
  }
  clock_ = node->get_clock();
  const std::string parameter_prefix = dwb_plugin_name_ + "." + name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "reference_topic",
    rclcpp::ParameterValue(std::string("virtual_spacecraft/odom")));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "reference_timeout", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "linear_feedback_gain", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "angular_feedback_gain", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "linear_x_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "linear_y_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_prefix + "angular_weight", rclcpp::ParameterValue(1.0));

  std::string reference_topic;
  node->get_parameter(parameter_prefix + "reference_topic", reference_topic);
  node->get_parameter(parameter_prefix + "reference_timeout", reference_timeout_);
  node->get_parameter(parameter_prefix + "linear_feedback_gain", linear_feedback_gain_);
  node->get_parameter(parameter_prefix + "angular_feedback_gain", angular_feedback_gain_);
  node->get_parameter(parameter_prefix + "linear_x_weight", linear_x_weight_);
  node->get_parameter(parameter_prefix + "linear_y_weight", linear_y_weight_);
  node->get_parameter(parameter_prefix + "angular_weight", angular_weight_);

  if (!std::isfinite(reference_timeout_) || reference_timeout_ <= 0.0 ||
    !std::isfinite(linear_feedback_gain_) || linear_feedback_gain_ < 0.0 ||
    !std::isfinite(angular_feedback_gain_) || angular_feedback_gain_ < 0.0 ||
    !std::isfinite(linear_x_weight_) || linear_x_weight_ < 0.0 ||
    !std::isfinite(linear_y_weight_) || linear_y_weight_ < 0.0 ||
    !std::isfinite(angular_weight_) || angular_weight_ < 0.0)
  {
    throw std::invalid_argument("Velocity critic gains, weights, and timeout are invalid");
  }

  reference_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
    reference_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&VirtualVelocityCritic::reference_callback, this, std::placeholders::_1));
}

bool VirtualVelocityCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  nav_msgs::msg::Odometry reference;
  rclcpp::Time reference_time;
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    if (!has_reference_) {
      target_is_valid_ = false;
      return false;
    }
    reference = latest_reference_;
    reference_time = last_reference_time_;
  }

  const double age = (clock_->now() - reference_time).seconds();
  if (!velocity_reference_is_valid(
      age,
      reference_timeout_,
      reference.header.frame_id,
      costmap_ros_->getGlobalFrameID()))
  {
    target_is_valid_ = false;
    return false;
  }

  target_ = make_velocity_target(
    reference.twist.twist.linear.x,
    reference.twist.twist.linear.y,
    tf2::getYaw(reference.pose.pose.orientation),
    reference.twist.twist.angular.z,
    pose.theta,
    velocity,
    linear_feedback_gain_,
    angular_feedback_gain_);
  if (!std::isfinite(target_.x) || !std::isfinite(target_.y) ||
    !std::isfinite(target_.theta))
  {
    target_is_valid_ = false;
    return false;
  }
  target_is_valid_ = true;
  return true;
}

double VirtualVelocityCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!target_is_valid_) {
    throw dwb_core::IllegalTrajectoryException(name_, "Virtual velocity reference is invalid");
  }
  return velocity_target_score(
    trajectory.velocity,
    target_,
    linear_x_weight_,
    linear_y_weight_,
    angular_weight_);
}

void VirtualVelocityCritic::reference_callback(
  const nav_msgs::msg::Odometry::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(reference_mutex_);
  latest_reference_ = *message;
  last_reference_time_ = clock_->now();
  has_reference_ = true;
}

}  // namespace swarm_dwb_critics

PLUGINLIB_EXPORT_CLASS(
  swarm_dwb_critics::VirtualVelocityCritic,
  dwb_core::TrajectoryCritic)
