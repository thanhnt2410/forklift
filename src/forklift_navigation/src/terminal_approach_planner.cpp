// SPDX-License-Identifier: Apache-2.0
#include <mutex>
#include <nav2_core/exceptions.hpp>
#include <nav2_smac_planner/smac_planner_hybrid.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "forklift_navigation/terminal_path.hpp"
namespace forklift_navigation {
class TerminalApproachPlanner : public nav2_core::GlobalPlanner {
  std::unique_ptr<nav2_smac_planner::SmacPlannerHybrid> curve_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  nav_msgs::msg::Path cached_;
  geometry_msgs::msg::PoseStamped cached_goal_;
  std::mutex plan_mutex_;
  double length_{.5};
  rclcpp::Clock::SharedPtr clock_;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    auto node=parent.lock();if(!node)throw std::runtime_error("Planner lifecycle node unavailable");
    nav2_util::declare_parameter_if_not_declared(node,name+".terminal_distance",rclcpp::ParameterValue(.5));
    node->get_parameter(name+".terminal_distance",length_);
    if(!std::isfinite(length_) || length_<.2 || length_>2.)throw std::runtime_error("Invalid terminal_distance");
    clock_=node->get_clock();costmap_=map;curve_=std::make_unique<nav2_smac_planner::SmacPlannerHybrid>();
    curve_->configure(parent,name,tf,map);
  }
  void activate() override {curve_->activate();}
  void deactivate() override {curve_->deactivate();cached_.poses.clear();}
  void cleanup() override {curve_->cleanup();curve_.reset();costmap_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override {
    std::lock_guard<std::mutex> plan_lock(plan_mutex_);
    const double a=tf2::getYaw(goal.pose.orientation),c=std::cos(a),s=std::sin(a);
    // Commit to the same valid route across replans. Small odometry changes
    // must not switch between different maneuvers on every planner tick.
    if(!cached_.poses.empty() && goal.header.frame_id==cached_goal_.header.frame_id &&
      std::hypot(goal.pose.position.x-cached_goal_.pose.position.x,goal.pose.position.y-cached_goal_.pose.position.y)<1e-5 &&
      std::abs(wrap_angle(a-tf2::getYaw(cached_goal_.pose.orientation)))<1e-5) {
      size_t closest=0;double best=1e9,travelled=0.;
      for(size_t i=0;i<cached_.poses.size();++i) {
        if(i)travelled+=std::hypot(cached_.poses[i].pose.position.x-cached_.poses[i-1].pose.position.x,
                                   cached_.poses[i].pose.position.y-cached_.poses[i-1].pose.position.y);
        if(travelled>.5)break;
        const auto & pose=cached_.poses[i].pose;
        const double d=std::hypot(pose.position.x-start.pose.position.x,pose.position.y-start.pose.position.y);
        if(d<best){best=d;closest=i;}
      }
      // A yaw tracking error at a gear change is not a reason to generate a
      // new maneuver back to the corridor entry. Keep the valid route while
      // the robot is close enough for its controller to recover.
      if(best<.20) {
        auto remaining=cached_;remaining.poses.erase(remaining.poses.begin(),remaining.poses.begin()+closest);
        // Keep the original tangent and gear cusp. Replacing the first pose
        // with the robot's off-path pose invents a short opposite segment,
        // which RPP interprets as another forward/reverse cusp.
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getCostmap()->getMutex());
        if(path_collision_free(remaining,costmap_->getCostmap(),costmap_->getRobotFootprint())) {
          remaining.header=goal.header;remaining.header.stamp=clock_->now();
          for(auto & pose:remaining.poses)pose.header=remaining.header;
          cached_=remaining;return remaining;
        }
      }
    }
    const double dx=goal.pose.position.x-start.pose.position.x,dy=goal.pose.position.y-start.pose.position.y;
    const double along=dx*c+dy*s,cross=-dx*s+dy*c;
    const double step=std::min(.02,costmap_->getCostmap()->getResolution()/2);
    nav_msgs::msg::Path result;
    // Replanning on the final leg must not send the robot back to its entry.
    if(along>=-.02 && along<=length_+.10 && std::abs(cross)<.08 &&
      std::abs(wrap_angle(tf2::getYaw(start.pose.orientation)-a))<.30) {
      result=terminal_leg(start,goal,step);
    } else {
      auto entry=goal;entry.pose.position.x-=length_*c;entry.pose.position.y-=length_*s;
      result=curve_->createPlan(start,entry);
      if(result.poses.empty())throw nav2_core::PlannerException("No route to terminal corridor entry");
      // Smac's grid endpoint may be displaced by one cell. Snap it to the
      // exact corridor entry, then check all interpolated poses with footprint.
      result.poses.back()=entry;
      auto tail=terminal_leg(entry,goal,step);
      result.poses.insert(result.poses.end(),tail.poses.begin()+1,tail.poses.end());
    }
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getCostmap()->getMutex());
    if(!path_collision_free(result,costmap_->getCostmap(),costmap_->getRobotFootprint()))
      throw nav2_core::PlannerException("Goal-aligned approach corridor or connecting path is blocked");
    result.header=goal.header;result.header.stamp=clock_->now();
    for(auto & pose:result.poses)pose.header=result.header;
    cached_=result;cached_goal_=goal;
    return result;
  }
};
}
PLUGINLIB_EXPORT_CLASS(forklift_navigation::TerminalApproachPlanner,nav2_core::GlobalPlanner)
