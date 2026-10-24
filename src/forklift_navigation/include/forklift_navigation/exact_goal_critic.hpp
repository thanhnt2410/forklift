// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"

namespace forklift_navigation
{
// Break grid-distance ties near the goal using continuous distance in metres.
// The existing path and footprint critics retain path/collision constraints.
class ExactGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D &) override
  {
    goal_ = goal;
    return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (trajectory.poses.empty()) {
      throw dwb_core::IllegalTrajectoryException("ExactGoal", "Empty trajectory");
    }
    const auto & end = trajectory.poses.back();
    return std::hypot(goal_.x - end.x, goal_.y - end.y);
  }
private:
  geometry_msgs::msg::Pose2D goal_;
};
}  // namespace forklift_navigation
