// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <limits>
#include <nav2_core/exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "forklift_navigation/terminal_path.hpp"
namespace forklift_navigation {
// RPP's built-in rotate_to_heading cannot be enabled together with reversing.
// Recover the vehicle heading while stationary, using the path quaternion
// (which remains the vehicle heading even for a reverse segment).
class StableReversingController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
  using Base=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
  enum Mode {TRACK,BRAKE,ROTATE,SETTLE};
  Mode mode_{TRACK};
  double target_yaw_{0.};
  geometry_msgs::msg::PoseStamped last_goal_;
  bool have_goal_{false};
public:
  void setPlan(const nav_msgs::msg::Path & path) override {
    if(!path.poses.empty()) {
      const auto & goal=path.poses.back();
      if(!have_goal_ || std::hypot(goal.pose.position.x-last_goal_.pose.position.x,goal.pose.position.y-last_goal_.pose.position.y)>.01 ||
        std::abs(wrap_angle(tf2::getYaw(goal.pose.orientation)-tf2::getYaw(last_goal_.pose.orientation)))>.01)mode_=TRACK;
      last_goal_=goal;have_goal_=true;
    }
    Base::setPlan(path);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override {
    if(global_plan_.poses.empty())return Base::computeVelocityCommands(pose,velocity,checker);
    geometry_msgs::msg::PoseStamped robot;
    if(!transformPose(global_plan_.header.frame_id,pose,robot))throw nav2_core::PlannerException("Heading recovery transform unavailable");
    const auto & goal=global_plan_.poses.back().pose;
    double tolerance=.035;
    geometry_msgs::msg::Pose pose_tol;geometry_msgs::msg::Twist velocity_tol;
    if(checker && checker->getTolerances(pose_tol,velocity_tol) && pose_tol.position.x>0.)tolerance=pose_tol.position.x;
    const double goal_distance=std::hypot(goal.position.x-robot.pose.position.x,goal.position.y-robot.pose.position.y);
    size_t nearest=0;double best=std::numeric_limits<double>::max(),distance=0.;
    for(size_t i=0;i<global_plan_.poses.size();++i) {
      if(i)distance+=std::hypot(global_plan_.poses[i].pose.position.x-global_plan_.poses[i-1].pose.position.x,
                               global_plan_.poses[i].pose.position.y-global_plan_.poses[i-1].pose.position.y);
      if(distance>max_robot_pose_search_dist_)break;
      const auto & p=global_plan_.poses[i].pose;
      const double d=std::hypot(p.position.x-robot.pose.position.x,p.position.y-robot.pose.position.y);
      if(d<best){best=d;nearest=i;}
    }
    const bool at_goal=goal_distance<=tolerance;
    const double desired=tf2::getYaw(at_goal?goal.orientation:global_plan_.poses[nearest].pose.orientation);
    const double robot_yaw=tf2::getYaw(robot.pose.orientation);
    if(mode_==TRACK && std::abs(wrap_angle(desired-robot_yaw))>(at_goal?.04:.40)) {
      mode_=BRAKE;target_yaw_=desired;
      RCLCPP_INFO(logger_,"Heading recovery: stop translation before correcting %.3f rad",wrap_angle(desired-robot_yaw));
    }
    if(mode_==TRACK)return Base::computeVelocityCommands(pose,velocity,checker);
    geometry_msgs::msg::TwistStamped command;command.header=pose.header;command.header.stamp=clock_->now();
    const double error=wrap_angle(target_yaw_-robot_yaw);
    if(mode_==BRAKE) {
      if(std::hypot(velocity.linear.x,velocity.linear.y)<.008 && std::abs(velocity.angular.z)<.02)mode_=ROTATE;
      return command;
    }
    if(mode_==SETTLE) {
      if(std::abs(velocity.angular.z)<.015) {
        if(std::abs(error)>.045)mode_=ROTATE;
        else {mode_=TRACK;RCLCPP_INFO(logger_,"Heading recovery complete; resume path");}
      }
      return command;
    }
    if(std::abs(error)<.02){mode_=SETTLE;return command;}
    const double target=std::clamp(1.2*error,-.18,.18);
    command.twist.angular.z=std::clamp(target,velocity.angular.z-max_angular_accel_*control_duration_,velocity.angular.z+max_angular_accel_*control_duration_);
    // Use RPP's footprint projection for every recovery command as well.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    if(use_collision_detection_ && isCollisionImminent(pose,0.,command.twist.angular.z,1.))
      throw nav2_core::PlannerException("Heading recovery footprint collision");
    return command;
  }
};
}
PLUGINLIB_EXPORT_CLASS(forklift_navigation::StableReversingController,nav2_core::Controller)
