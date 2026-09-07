#ifndef SWARM_DWB_CRITICS__VIRTUAL_VELOCITY_CRITIC_HPP_
#define SWARM_DWB_CRITICS__VIRTUAL_VELOCITY_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "dwb_core/trajectory_critic.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace swarm_dwb_critics
{

struct VelocityTarget
{
  double x;
  double y;
  double theta;
};

VelocityTarget make_velocity_target(
  double reference_body_x,
  double reference_body_y,
  double reference_yaw,
  double reference_yaw_rate,
  double robot_yaw,
  const nav_2d_msgs::msg::Twist2D & measured_velocity,
  double linear_feedback_gain,
  double angular_feedback_gain);

double velocity_target_score(
  const nav_2d_msgs::msg::Twist2D & candidate,
  const VelocityTarget & target,
  double linear_x_weight,
  double linear_y_weight,
  double angular_weight);

bool velocity_reference_is_valid(
  double age,
  double timeout,
  const std::string & reference_frame,
  const std::string & costmap_frame);

class VirtualVelocityCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  void reference_callback(const nav_msgs::msg::Odometry::SharedPtr message);

  std::mutex reference_mutex_;
  nav_msgs::msg::Odometry latest_reference_;
  rclcpp::Time last_reference_time_;
  bool has_reference_{false};
  bool target_is_valid_{false};
  VelocityTarget target_{0.0, 0.0, 0.0};
  double reference_timeout_{0.25};
  double linear_feedback_gain_{0.0};
  double angular_feedback_gain_{0.0};
  double linear_x_weight_{1.0};
  double linear_y_weight_{1.0};
  double angular_weight_{1.0};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr reference_subscription_;
};

}  // namespace swarm_dwb_critics

#endif  // SWARM_DWB_CRITICS__VIRTUAL_VELOCITY_CRITIC_HPP_
