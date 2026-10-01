#include "obca_navigation/core.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool value,const char *message) {if(!value)throw std::runtime_error(message);}
obca::Grid corridor(double yaw,double half_width,int boundary) {
  obca::Grid g;g.width=g.height=121;g.resolution=0.1;g.x0=g.y0=-6.05;
  g.cells.assign(g.width*g.height,boundary);
  for(int i=0;i<static_cast<int>(g.cells.size());++i) {
    const auto p=g.center(i);
    const double x=std::cos(yaw)*p.x+std::sin(yaw)*p.y;
    const double y=-std::sin(yaw)*p.x+std::cos(yaw)*p.y;
    // Observed L corridor; no prior map, unknown space remains unknown.
    if((x>-1 && x<2+half_width && std::abs(y)<half_width) ||
       (std::abs(x-2)<half_width && y>-half_width && y<4))g.cells[i]=0;
  }
  return g;
}
}
int main()try {
  using namespace obca;
  Config c;c.margin=0.12;c.reference_clearance=0.35;c.goal_clearance_weight=0.5;
  c.reference_wall_weight=1.0;c.validate();
  GoalFeatures straight;straight.observed_connected=true;straight.progress_m=2.4;
  straight.path_cost_m=2.4;straight.goal_clearance_m=0.4;straight.route_clearance_m=0.35;
  const auto dead_end=scoreLocalGoal(straight,c);
  GoalFeatures corner=straight;corner.progress_m=2.0;corner.path_cost_m=2.2;
  corner.goal_clearance_m=0.7;corner.route_clearance_m=0.4;
  corner.lateral_m=1.0;corner.heading_change_rad=std::acos(-1.0)/2;
  corner.continuation_m=c.goal_continuation_distance;
  const auto bend=scoreLocalGoal(corner,c);
  check(bend.eligible && bend.total>dead_end.total,"open bend lost to a longer forward dead end");
  auto unsafe=corner;unsafe.observed_connected=false;unsafe.progress_m=1e6;
  check(!scoreLocalGoal(unsafe,c).eligible,"unknown/disconnected candidate was scored as valid");
  unsafe=corner;unsafe.route_clearance_m=std::numeric_limits<double>::quiet_NaN();
  check(!scoreLocalGoal(unsafe,c).eligible,"nonfinite candidate was accepted");
  auto backwards=corner;backwards.initial_heading_error_rad=std::acos(-1.0);
  check(scoreLocalGoal(backwards,c).total<bend.total,"opposite route entrance is not penalized");
  auto switch_goal=corner;switch_goal.has_previous=true;switch_goal.previous_goal_distance_m=2;
  check(scoreLocalGoal(switch_goal,c).total<bend.total,"target switching has no continuity cost");
  auto bottleneck=corner;bottleneck.route_clearance_m=0.1;
  check(scoreLocalGoal(bottleneck,c).total<bend.total,"narrow route is not penalized");
  Config doubled=c;doubled.reference_distance*=2;doubled.reference_clearance*=2;
  doubled.goal_clearance_target*=2;doubled.goal_continuation_distance*=2;doubled.wheelbase*=2;
  auto scaled=corner;scaled.progress_m*=2;scaled.path_cost_m*=2;scaled.lateral_m*=2;
  scaled.goal_clearance_m*=2;scaled.route_clearance_m*=2;scaled.continuation_m*=2;
  check(std::abs(scoreLocalGoal(scaled,doubled).total-bend.total)<1e-9,
    "score changes just because geometric units/horizon scale");
  for(int boundary : {-1,100})for(double width : {0.5,0.8})for(double yaw : {0.0,0.4,1.57}) {
    Config turn=c;turn.reference_distance=3.0;
    auto g=corridor(yaw,width,boundary);
    Pose ego{0.8*std::cos(yaw),0.8*std::sin(yaw),yaw,0};
    check(g.footprint(ego,turn),"test corridor start is unsafe");
    GoalEvaluation selected;const auto route=reference(g,ego,turn,{},&selected);
    check(route.size()>2 && selected.eligible,"observed corner has no scored goal");
    const auto &end=route.back();
    const double forward=std::cos(yaw)*end.x+std::sin(yaw)*end.y;
    const double side=-std::sin(yaw)*end.x+std::cos(yaw)*end.y;
    std::cout<<"corner width="<<2*width<<" yaw="<<yaw<<" end="<<forward<<","<<side
      <<" score="<<selected.total<<" continuation="<<selected.features.continuation_m<<'\n';
    check(side>0.7,"local goal keeps driving into the forward wall instead of turning");
    check(std::abs(forward-2)<width,"goal leaves the connected corridor");
    check(distance(selected.goal,end)<1e-8,"score does not describe the returned goal");
    check(g.footprint(selected.goal,turn),"scored goal body enters occupied/unknown space");
    double arc=0;for(std::size_t i=1;i<route.size();++i)arc+=distance(route[i-1],route[i]);
    check(arc<=turn.reference_distance+1e-8,"scoring exceeds geometric search budget");
    const auto repeated=reference(g,ego,turn,{},&selected);
    check(distance(repeated.back(),end)<1e-8,"identical inputs cause a goal switch");
  }
  // Disabling reward terms must not make all safe candidates disappear merely
  // because their valid scores are negative.
  Config penalty_only=c;penalty_only.goal_progress_weight=0;penalty_only.goal_forward_weight=0;
  penalty_only.goal_clearance_weight=0;penalty_only.goal_continuation_weight=0;penalty_only.validate();
  GoalEvaluation negative;
  check(!reference(corridor(0,0.8,100),Pose{0.8,0,0,0},penalty_only,{},&negative).empty() &&
    negative.eligible && negative.total<0,"negative safe scores incorrectly suppress all routes");
  auto blocked=corridor(0,0.5,-1);std::fill(blocked.cells.begin(),blocked.cells.end(),-1);
  GoalEvaluation absent;
  check(reference(blocked,Pose{},c,{},&absent).empty() && !absent.eligible,
    "scoring converted unseen space into a route");
  std::cout<<"local goal scoring checks passed\n";
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
