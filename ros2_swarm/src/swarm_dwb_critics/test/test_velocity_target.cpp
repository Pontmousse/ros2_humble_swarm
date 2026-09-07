#include <cmath>

#include "gtest/gtest.h"
#include "swarm_dwb_critics/virtual_velocity_critic.hpp"

using swarm_dwb_critics::make_velocity_target;
using swarm_dwb_critics::velocity_reference_is_valid;
using swarm_dwb_critics::velocity_target_score;

TEST(VelocityTarget, RotatesReferenceIntoRobotBodyFrame)
{
  nav_2d_msgs::msg::Twist2D measured;
  const double half_pi = std::acos(-1.0) / 2.0;
  const auto target = make_velocity_target(
    1.0, 0.0, half_pi, 0.2, 0.0, measured, 0.0, 0.0);
  EXPECT_NEAR(target.x, 0.0, 1.0e-9);
  EXPECT_NEAR(target.y, 1.0, 1.0e-9);
  EXPECT_NEAR(target.theta, 0.2, 1.0e-9);
}

TEST(VelocityTarget, RotatesWorldVelocityIntoPhysicalBodyFrame)
{
  nav_2d_msgs::msg::Twist2D measured;
  const double half_pi = std::acos(-1.0) / 2.0;
  const auto target = make_velocity_target(
    1.0, 0.0, 0.0, 0.0, half_pi, measured, 0.0, 0.0);
  EXPECT_NEAR(target.x, 0.0, 1.0e-9);
  EXPECT_NEAR(target.y, -1.0, 1.0e-9);
}

TEST(VelocityTarget, AddsMeasuredVelocityFeedback)
{
  nav_2d_msgs::msg::Twist2D measured;
  measured.x = 0.2;
  measured.theta = 0.1;
  const auto target = make_velocity_target(
    0.4, 0.0, 0.0, 0.3, 0.0, measured, 0.5, 0.5);
  EXPECT_NEAR(target.x, 0.5, 1.0e-9);
  EXPECT_NEAR(target.y, 0.0, 1.0e-9);
  EXPECT_NEAR(target.theta, 0.4, 1.0e-9);
}

TEST(VelocityTarget, CorrectCandidateHasLowestScore)
{
  nav_2d_msgs::msg::Twist2D measured;
  const auto target = make_velocity_target(
    0.3, -0.1, 0.0, 0.2, 0.0, measured, 0.0, 0.0);
  nav_2d_msgs::msg::Twist2D correct;
  correct.x = target.x;
  correct.y = target.y;
  correct.theta = target.theta;
  nav_2d_msgs::msg::Twist2D wrong;

  EXPECT_DOUBLE_EQ(velocity_target_score(correct, target, 1.0, 1.0, 1.0), 0.0);
  EXPECT_GT(velocity_target_score(wrong, target, 1.0, 1.0, 1.0), 0.0);
}

TEST(VelocityTarget, ReferenceRequiresFreshMatchingNonemptyFrames)
{
  EXPECT_TRUE(velocity_reference_is_valid(0.1, 0.25, "/swarm_map", "swarm_map"));
  EXPECT_FALSE(velocity_reference_is_valid(0.3, 0.25, "swarm_map", "swarm_map"));
  EXPECT_FALSE(velocity_reference_is_valid(-0.1, 0.25, "swarm_map", "swarm_map"));
  EXPECT_FALSE(velocity_reference_is_valid(0.1, 0.25, "", "swarm_map"));
  EXPECT_FALSE(velocity_reference_is_valid(0.1, 0.25, "map", "swarm_map"));
}
