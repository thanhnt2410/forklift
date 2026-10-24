// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace forklift_navigation {
inline geometry_msgs::msg::Quaternion heading(double a) {
  geometry_msgs::msg::Quaternion q; q.z=std::sin(a/2);q.w=std::cos(a/2);return q;
}
inline double wrap_angle(double a) {return std::atan2(std::sin(a),std::cos(a));}
// A straight terminal leg on the goal's axis. A small tracking error at replan
// time is joined with a tangent-continuous Bezier before the straight remainder.
inline nav_msgs::msg::Path terminal_leg(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, double step)
{
  nav_msgs::msg::Path path;path.header=goal.header;
  const double a=tf2::getYaw(goal.pose.orientation),c=std::cos(a),s=std::sin(a);
  const double dx=goal.pose.position.x-start.pose.position.x,dy=goal.pose.position.y-start.pose.position.y;
  const double remaining=dx*c+dy*s;
  const double lateral=-dx*s+dy*c;
  const double sa=tf2::getYaw(start.pose.orientation);
  auto add=[&](double x,double y,double angle) {
    auto p=goal;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation=heading(angle);path.poses.push_back(p);
  };
  double from=remaining;
  if (remaining>.04 && (std::abs(lateral)>.002 || std::abs(wrap_angle(sa-a))>.02)) {
    const double blend=std::min(.25,remaining*.5);
    const double ex=goal.pose.position.x-(remaining-blend)*c;
    const double ey=goal.pose.position.y-(remaining-blend)*s;
    const double h=std::hypot(ex-start.pose.position.x,ey-start.pose.position.y)/3.;
    const double x0=start.pose.position.x,y0=start.pose.position.y;
    const double x1=x0+h*std::cos(sa),y1=y0+h*std::sin(sa);
    const double x2=ex-h*c,y2=ey-h*s;
    const int n=std::max(2,int(std::ceil(blend/step)));
    for(int i=0;i<n;++i) {
      const double t=double(i)/n,u=1-t;
      const double x=u*u*u*x0+3*u*u*t*x1+3*u*t*t*x2+t*t*t*ex;
      const double y=u*u*u*y0+3*u*u*t*y1+3*u*t*t*y2+t*t*t*ey;
      const double vx=3*u*u*(x1-x0)+6*u*t*(x2-x1)+3*t*t*(ex-x2);
      const double vy=3*u*u*(y1-y0)+6*u*t*(y2-y1)+3*t*t*(ey-y2);
      add(x,y,std::atan2(vy,vx));
    }
    from=remaining-blend;
  }
  const int n=std::max(1,int(std::ceil(std::max(0.,from)/step)));
  for(int i=0;i<=n;++i) {
    const double d=std::max(0.,from)*(1-double(i)/n);
    add(goal.pose.position.x-d*c,goal.pose.position.y-d*s,a);
  }
  return path;
}
inline bool path_collision_free(const nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D * map,const nav2_costmap_2d::Footprint & footprint)
{
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
  for(size_t i=0;i<path.poses.size();++i) {
    const auto & p=path.poses[i].pose;
    const auto & prev=path.poses[i?i-1:i].pose;
    const double distance=std::hypot(p.position.x-prev.position.x,p.position.y-prev.position.y);
    const double old_yaw=tf2::getYaw(prev.orientation),delta=wrap_angle(tf2::getYaw(p.orientation)-old_yaw);
    const int n=std::max({1,int(std::ceil(distance/(map->getResolution()/2))),int(std::ceil(std::abs(delta)/.025))});
    for(int j=0;j<=n;++j) {
      const double t=double(j)/n,x=prev.position.x+t*(p.position.x-prev.position.x),y=prev.position.y+t*(p.position.y-prev.position.y);
      unsigned int mx,my;
      if(!map->worldToMap(x,y,mx,my) || map->getCost(mx,my)>=253)return false;
      const double cost=checker.footprintCostAtPose(x,y,old_yaw+t*delta,footprint);
      // INSCRIBED (253) is inflated cost, not occupied geometry at a
      // footprint edge. Reject lethal/unknown; center inscribed stays blocked.
      if(cost<0 || cost>=254)return false;
    }
  }
  return true;
}
}
