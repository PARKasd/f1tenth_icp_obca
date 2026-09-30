#pragma once
#include "obca_navigation/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace obca {
inline Config parameters(rclcpp::Node &node) {
  Config c;
#define PARAM(field) c.field=node.declare_parameter(#field,c.field)
  PARAM(horizon);PARAM(max_obstacles);PARAM(max_iterations);PARAM(dt);PARAM(wheelbase);
  PARAM(front);PARAM(rear);PARAM(half_width);PARAM(margin);PARAM(max_speed);PARAM(max_steering);
  PARAM(max_steering_rate);PARAM(max_accel);PARAM(max_decel);PARAM(max_lateral_accel);
  PARAM(solve_seconds);PARAM(tolerance);PARAM(validation_tolerance);PARAM(position_weight);
  PARAM(heading_weight);PARAM(speed_weight);PARAM(steering_weight);PARAM(acceleration_weight);
  PARAM(smooth_weight);PARAM(grid_resolution);PARAM(map_radius);PARAM(map_ttl);
  PARAM(reference_distance);PARAM(reference_clearance);PARAM(validation_step);
  PARAM(goal_forward_weight);PARAM(goal_lateral_weight);PARAM(goal_path_weight);PARAM(goal_continuity_weight);
#undef PARAM
  c.validate();return c;
}
inline bool poseFrom(const geometry_msgs::msg::Pose &message,Pose &out) {
  const auto&q=message.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(norm+message.position.x+message.position.y) || norm<0.9 || norm>1.1)return false;
  out.x=message.position.x;out.y=message.position.y;
  out.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),norm-2*(q.y*q.y+q.z*q.z));return true;
}
inline geometry_msgs::msg::Pose poseMessage(const Pose&p) {
  geometry_msgs::msg::Pose m;m.position.x=p.x;m.position.y=p.y;
  m.orientation.z=std::sin(p.yaw/2);m.orientation.w=std::cos(p.yaw/2);return m;
}
using Steady=std::chrono::steady_clock;
inline double age(Steady::time_point t) {return std::chrono::duration<double>(Steady::now()-t).count();}
inline bool quality(const diagnostic_msgs::msg::DiagnosticArray &msg,double min_inlier,double max_residual) {
  for(const auto &s:msg.status) {
    double inlier=-1,residual=std::numeric_limits<double>::infinity(),dead=1;
    bool converged=false,rejected=true,impermissible=true;
    try {for(const auto &kv:s.values) {
      if(kv.key=="inlier_ratio")inlier=std::stod(kv.value);
      if(kv.key=="residual_rms")residual=std::stod(kv.value);
      if(kv.key=="dead_reckoning_sec")dead=std::stod(kv.value);
      if(kv.key=="converged")converged=kv.value=="true";
      if(kv.key=="gate_rejected")rejected=kv.value!="false";
      if(kv.key=="pose_impermissible")impermissible=kv.value!="false";
    }}catch(const std::exception&){return false;}
    if(s.level<2 && converged && !rejected && !impermissible && std::isfinite(inlier+residual+dead) &&
       inlier>=min_inlier && residual>=0 && residual<=max_residual && dead==0)return true;
  }
  return false;
}
} // namespace obca
