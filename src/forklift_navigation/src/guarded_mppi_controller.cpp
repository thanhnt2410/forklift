// SPDX-License-Identifier: Apache-2.0
// MPPI lifecycle/optimizer sequencing follows Nav2 1.1.20, copyright (c) 2022
// Samsung Research America, @artofnothingness Alexey Budyakov (Apache-2.0).
#include <chrono>
#include <nav2_core/exceptions.hpp>
#include <nav2_mppi_controller/controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "forklift_navigation/route_guard.hpp"

namespace forklift_navigation {
// Opt-in MPPI adapter for a fixed, approved route. A blocked reference is a
// failure, never permission for MPPI to choose an alternate local route.
class GuardedMPPIController : public nav2_mppi_controller::MPPIController {
  using Base=nav2_mppi_controller::MPPIController;
  static constexpr double corridor_=0.12;
  enum Mode {TRACK,BRAKE,ROTATE,SETTLE};
  Mode mode_{TRACK};
  double target_yaw_{0.};
  geometry_msgs::msg::PoseStamped final_goal_;
  bool reset_after_recovery_{false};
public:
  void setPlan(const nav_msgs::msg::Path & path) override {
    mode_=TRACK;reset_after_recovery_=false;
    if(!path.poses.empty()) {final_goal_=path.poses.back();final_goal_.header=path.header;}
    reset();  // Do not warm-start a new FollowPath leg with the previous gear.
    Base::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override {
    const auto started=std::chrono::steady_clock::now();
    if(reset_after_recovery_ || clock_->now()-last_time_called_>rclcpp::Duration::from_seconds(reset_period_)) {
      reset();reset_after_recovery_=false;
    }
    last_time_called_=clock_->now();
    std::lock_guard<std::mutex> parameters_lock(*parameters_handler_->getLock());
    const auto reference=path_handler_.transformPath(pose);
    auto * map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*map->getMutex());
    const auto footprint=costmap_ros_->getRobotFootprint();
    if(reference.poses.empty() || !path_collision_free(route_prefix(reference,.45),map,footprint))
      throw nav2_core::PlannerException("MPPI approved reference blocked; no detour allowed");
    if(route_distance(pose.pose.position.x,pose.pose.position.y,reference)>corridor_)
      throw nav2_core::PlannerException("MPPI robot outside approved route corridor");

    // A cusp uses inversion tolerance, but the final goal needs the stricter
    // goal-checker position tolerance before an in-place heading correction.
    // Rotating too early at a final goal can strand a lateral position error.
    auto goal=final_goal_;goal.header.stamp=reference.header.stamp;
    geometry_msgs::msg::PoseStamped local_goal;
    try {local_goal=tf_buffer_->transform(goal,reference.header.frame_id);}
    catch(const tf2::TransformException & error) {throw nav2_core::PlannerException(error.what());}
    const auto & end=reference.poses.back().pose.position;
    const bool final_endpoint=std::hypot(end.x-local_goal.pose.position.x,end.y-local_goal.pose.position.y)<.001;
    double correction_radius=.07;
    if(final_endpoint) {
      geometry_msgs::msg::Pose tolerance;geometry_msgs::msg::Twist velocity_tolerance;
      correction_radius=checker && checker->getTolerances(tolerance,velocity_tolerance) && tolerance.position.x>0. ?
        std::min(correction_radius,.8*tolerance.position.x):.02;
    }
    double heading_error=0.;
    if(mode_==TRACK && endpoint_heading_correction(pose.pose,reference,heading_error,correction_radius)) {
      target_yaw_=tf2::getYaw(reference.poses.back().pose.orientation);mode_=BRAKE;
      RCLCPP_INFO(logger_,"MPPI endpoint heading recovery: brake before correcting %.3f rad",heading_error);
    }
    if(mode_!=TRACK) {
      geometry_msgs::msg::TwistStamped command;command.header=pose.header;command.header.stamp=clock_->now();
      const double error=wrap_angle(target_yaw_-tf2::getYaw(pose.pose.orientation));
      if(mode_==BRAKE) {
        if(std::hypot(speed.linear.x,speed.linear.y)<.008 && std::abs(speed.angular.z)<.02)mode_=ROTATE;
        return command;
      }
      if(mode_==SETTLE) {
        if(std::abs(speed.angular.z)<.015) {
          if(std::abs(error)>.045)mode_=ROTATE;
          else {mode_=TRACK;reset_after_recovery_=true;RCLCPP_INFO(logger_,"MPPI endpoint heading recovered; resume selected path");}
        }
        return command;
      }
      if(std::abs(error)<.02){mode_=SETTLE;return command;}
      command.twist.angular.z=std::clamp(1.2*error,-.18,.18);
      nav_msgs::msg::Path sweep;sweep.header=pose.header;sweep.poses.push_back(pose);
      for(int i=1;i<=12;++i) {
        auto point=pose;
        point.pose.orientation=heading(tf2::getYaw(pose.pose.orientation)+command.twist.angular.z*.05*i);
        sweep.poses.push_back(point);
      }
      if(!path_collision_free(sweep,map,footprint))
        throw nav2_core::PlannerException("MPPI endpoint heading recovery footprint collision");
      return command;
    }

    auto command=optimizer_.evalControl(pose,speed,reference,checker);
    if(!limit_route_command(command.twist))
      throw nav2_core::PlannerException("MPPI command is nonfinite or requests lateral travel");
    const auto trajectory=optimizer_.getOptimizedTrajectory();
    nav_msgs::msg::Path prediction;prediction.header=reference.header;
    prediction.poses.push_back(pose);
    for(size_t i=0;i<trajectory.shape(0);++i) {
      const double x=trajectory(i,0),y=trajectory(i,1),yaw=trajectory(i,2);
      if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        route_distance(x,y,reference)>corridor_)
        throw nav2_core::PlannerException("MPPI prediction outside approved route corridor");
      geometry_msgs::msg::PoseStamped p;p.header=reference.header;
      p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation=heading(yaw);
      prediction.poses.push_back(p);
    }
    if(!path_collision_free(prediction,map,footprint))
      throw nav2_core::PlannerException("MPPI selected prediction footprint collision");
    if(visualize_)visualize(reference);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    RCLCPP_INFO_THROTTLE(logger_,*clock_,1000,"Guarded MPPI compute %.3f ms",ms);
    return command;
  }
};
}  // namespace forklift_navigation
PLUGINLIB_EXPORT_CLASS(forklift_navigation::GuardedMPPIController,nav2_core::Controller)
