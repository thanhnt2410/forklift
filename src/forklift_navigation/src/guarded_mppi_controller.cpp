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
public:
  void setPlan(const nav_msgs::msg::Path & path) override {
    reset();  // Do not warm-start a new FollowPath leg with the previous gear.
    Base::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override {
    const auto started=std::chrono::steady_clock::now();
    if(clock_->now()-last_time_called_>rclcpp::Duration::from_seconds(reset_period_))reset();
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
