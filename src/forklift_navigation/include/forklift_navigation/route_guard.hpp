// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <limits>
#include <geometry_msgs/msg/twist.hpp>
#include "forklift_navigation/terminal_path.hpp"

namespace forklift_navigation {
// Use the vehicle quaternion, including on a reverse leg. The direction of
// travel at a cusp is not the required vehicle heading.
inline bool endpoint_heading_correction(
  const geometry_msgs::msg::Pose & pose,const nav_msgs::msg::Path & reference,double & error,
  double position_tolerance=.07)
{
  if(reference.poses.empty())return false;
  const auto & end=reference.poses.back().pose;
  error=wrap_angle(tf2::getYaw(end.orientation)-tf2::getYaw(pose.orientation));
  // Stay inside MPPI's 0.08 m inversion tolerance before rotating. Braking
  // outside that region can strand the robot beside a cusp: correcting yaw
  // alone cannot satisfy the outstanding lateral position error.
  return std::hypot(pose.position.x-end.position.x,pose.position.y-end.position.y)<position_tolerance && std::abs(error)>.08;
}
// Humble applies a Savitzky-Golay filter after optimizer constraints. Bound
// the filtered command as well, preserving curvature rather than clipping axes.
inline bool limit_route_command(geometry_msgs::msg::Twist & command) {
  const double v=command.linear.x,w=command.angular.z;
  if(!std::isfinite(v) || !std::isfinite(w) || !std::isfinite(command.linear.y) ||
    std::abs(command.linear.y)>1e-6)return false;
  double scale=1.;
  if(v>.25)scale=std::min(scale,.25/v);
  if(v<-.16)scale=std::min(scale,-.16/v);
  if(std::abs(w)>.6)scale=std::min(scale,.6/std::abs(w));
  command.linear.x*=scale;command.angular.z*=scale;
  return true;
}
// Distance to the active polyline, including clamped endpoints. No direction
// inference: MPPI's path handler owns inversion/progress and supplies this leg.
inline double route_distance(double x, double y, const nav_msgs::msg::Path & path) {
  double best=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<path.poses.size();++i) {
    const auto & a=path.poses[i?i-1:i].pose.position;
    const auto & b=path.poses[i].pose.position;
    const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
    const double t=l2>1e-12?std::clamp(((x-a.x)*dx+(y-a.y)*dy)/l2,0.,1.):0.;
    best=std::min(best,std::hypot(x-a.x-t*dx,y-a.y-t*dy));
  }
  return best;
}

inline nav_msgs::msg::Path route_prefix(const nav_msgs::msg::Path & path,double length) {
  nav_msgs::msg::Path out;out.header=path.header;
  double distance=0.;
  for(size_t i=0;i<path.poses.size();++i) {
    if(i) {
      const auto & a=path.poses[i-1].pose.position;
      const auto & b=path.poses[i].pose.position;
      distance+=std::hypot(b.x-a.x,b.y-a.y);
    }
    out.poses.push_back(path.poses[i]);
    if(distance>=length)break;
  }
  return out;
}
}  // namespace forklift_navigation
