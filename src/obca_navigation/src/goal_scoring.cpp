#include "obca_navigation/core.hpp"
#include <algorithm>

namespace obca {
GoalEvaluation scoreLocalGoal(const GoalFeatures &f,const Config &c) {
  c.validate();
  GoalEvaluation result;result.features=f;
  const double values[]={f.progress_m,f.lateral_m,f.path_cost_m,f.goal_clearance_m,
    f.route_clearance_m,f.initial_heading_error_rad,f.heading_change_rad,
    f.continuation_m,f.previous_goal_distance_m};
  for(double value:values)if(!std::isfinite(value))return result;
  if(!f.observed_connected || f.progress_m<=c.reference_clearance ||
     f.progress_m>c.reference_distance+1e-6 || f.path_cost_m<0 ||
     f.goal_clearance_m<0 || f.route_clearance_m<0 || f.continuation_m<0 ||
     f.previous_goal_distance_m<0)return result;
  const auto unit=[](double value){return std::clamp(value,0.0,1.0);};
  result.eligible=true;
  result.progress=c.goal_progress_weight*unit(f.progress_m/c.reference_distance);
  // Alignment of the beginning of the route, not the goal's bearing: a forward
  // entrance to a ninety-degree corridor should not be mistaken for lateral drift.
  result.direction=c.goal_forward_weight*(1+std::cos(f.initial_heading_error_rad))/2;
  result.lateral=-c.goal_lateral_weight*unit(std::abs(f.lateral_m)/c.reference_distance);
  result.path_cost=-c.goal_path_weight*f.path_cost_m/c.reference_distance;
  result.clearance=c.goal_clearance_weight*unit(f.goal_clearance_m/c.goal_clearance_target);
  const double proximity=c.reference_clearance/std::max(c.reference_clearance,f.route_clearance_m);
  result.route_clearance=-c.goal_route_clearance_weight*proximity*proximity;
  const double steering_ratio=c.wheelbase*std::abs(f.heading_change_rad)/
    (f.progress_m*std::tan(c.max_steering));
  result.turn=-c.goal_turn_weight*std::min(steering_ratio,2.0);
  result.continuation=c.goal_continuation_weight*unit(f.continuation_m/c.goal_continuation_distance);
  result.continuity=f.has_previous?-c.goal_continuity_weight*
    std::min(f.previous_goal_distance_m/c.reference_distance,2.0):0.0;
  result.total=result.progress+result.direction+result.lateral+result.path_cost+
    result.clearance+result.route_clearance+result.turn+result.continuation+result.continuity;
  return result;
}
} // namespace obca
