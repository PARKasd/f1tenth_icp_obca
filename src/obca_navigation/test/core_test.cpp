#include "obca_navigation/core.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
namespace obca { double derivativeError(); }
void check(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
obca::Grid grid() {obca::Grid g;g.width=g.height=121;g.resolution=0.1;g.x0=g.y0=-6.05;g.cells.assign(g.width*g.height,0);return g;}
int main()try {
  using namespace obca;
  Config c;c.solve_seconds=5;c.validate();Pose ego{};
  auto g=grid();
  check(derivativeError()<1e-6,"analytic Jacobian/gradient mismatch");
  check(g.footprint(ego,c),"empty-space footprint");
  g.cells[g.index(0.3,0.0)]=-1;check(!g.footprint(ego,c),"unknown space must block body");
  g=grid();g.cells[g.index(0.3,0)]=100;check(!g.footprint(ego,c),"occupied body accepted");
  g=grid();
  const auto ref=reference(g,ego,c);check(ref.size()>2,"forward reference missing");check(ref.back().x>1,"reference must advance");
  auto solved=solve(ego,0,ref,{},c);
  std::cout<<"straight: "<<solved.reason<<" "<<solved.elapsed_ms<<" ms\n";
  check(solved.success,"empty-space solver failed");
  check(solved.states.back().x>0.5,"solver did not progress");
  std::string reason;check(validatePath(g,solved.states,c,reason),"straight path invalid");
  check(std::abs(solved.states.back().v)<1e-6,"terminal stop constraint missing");
  const std::vector<Box> walls{{-1,0.7,4,0.9},{-1,-0.9,4,-0.7}};
  auto corridor=solve(ego,0,ref,walls,c);
  std::cout<<"corridor: "<<corridor.reason<<" "<<corridor.elapsed_ms<<" ms\n";
  check(corridor.success,"OBCA corridor solve failed");
  for(const auto&p:corridor.states)for(const auto&b:walls)check(!overlap(p,c.front,c.rear,c.half_width,b),"OBCA intersects wall");
  // A static obstacle on the original centreline: reference search must go around it,
  // and the optimized vehicle rectangle must clear it, not just its centre point.
  auto detour_grid=grid();
  for(int i=0;i<static_cast<int>(detour_grid.cells.size());++i){const auto p=detour_grid.center(i);
    if(p.x>=1.1 && p.x<=1.4 && std::abs(p.y)<=0.15)detour_grid.cells[i]=100;}
  const auto detour_ref=reference(detour_grid,ego,c);
  const auto detour_boxes=obstacles(detour_grid,ego,4);
  auto detour=solve(ego,0,detour_ref,detour_boxes,c);
  std::cout<<"detour: "<<detour.reason<<" "<<detour.elapsed_ms<<" ms\n";
  check(detour.success,"static obstacle detour solve failed");
  check(validatePath(detour_grid,detour.states,c,reason),"detour failed independent collision validation");
  auto blocked=solve(ego,0,ref,{{-0.2,-0.3,0.5,0.3}},c);
  check(!blocked.success && blocked.states.empty(),"infeasible initial collision accepted");
  std::vector<Box> excess(c.max_obstacles+1);check(!solve(ego,0,ref,excess,c).success,"obstacle overflow silently truncated");
  Config deadline=c;deadline.solve_seconds=1e-9;check(!solve(ego,0,ref,walls,deadline).success,"deadline ignored");
  auto unknown=grid();std::fill(unknown.cells.begin(),unknown.cells.end(),-1);
  check(reference(unknown,ego,c).empty(),"unknown grid planned through");
  check(!validatePath(unknown,solved.states,c,reason),"unknown swept path accepted");
  unknown.has_known_body=true;unknown.known_body=ego;
  check(unknown.footprint(ego,c),"exact physical body is not recognized in blind spot");
  Pose outside=ego;outside.y=0.02;
  check(!unknown.footprint(outside,c),"self-footprint exemption leaked into unknown space");
  LocalMap map(c);map.ray(0,0,2,0,1);map.hit(2,0,1);
  auto observed=map.snapshot(ego,1);check(observed.cells[observed.index(1,0)]==0,"ray free space missing");
  check(observed.cells[observed.index(2,0)]==100,"ray endpoint missing");
  observed=map.snapshot(ego,1+c.map_ttl+0.01);check(observed.cells[observed.index(1,0)]==-1,"expired free space retained");
  map.hit(1,0,10);map.clear();observed=map.snapshot(ego,10);check(observed.cells[observed.index(1,0)]==-1,"reset retained old map");
  // One-cell wall across the complete map must never be crossed by reference search.
  g=grid();for(int y=0;y<g.height;++y)g.cells[y*g.width+g.index(1,0)%g.width]=100;
  const auto stopped_ref=reference(g,ego,c);for(const auto&p:stopped_ref)check(p.x<1,"reference crosses disconnected wall");
  // Swept validation must detect a collision between two clear end poses.
  auto swept=solved.states;for(std::size_t i=0;i<swept.size();++i){swept[i].x=i?2.0:0;swept[i].y=0;swept[i].yaw=0;}
  check(!validatePath(g,swept,c,reason),"swept validator missed thin wall");
  // Synthetic 270-degree F1TENTH scan in a closed straight corridor.
  LocalMap lidar_map(c);
  std::vector<Pose> hits;
  for(int i=0;i<=1080;++i){const double a=(-135.0+i*0.25)*std::acos(-1.0)/180;
    const double dx=std::cos(a),dy=std::sin(a),ox=0.27;
    double r=6.0;
    if(dy>1e-9)r=std::min(r,0.8/dy);
    if(dy< -1e-9)r=std::min(r,-0.8/dy);
    if(dx>1e-9)r=std::min(r,(4-ox)/dx);
    Pose hit{ox+r*dx,r*dy,0,0};lidar_map.ray(ox,0,hit.x,hit.y,1);hits.push_back(hit);}
  for(const auto&p:hits)lidar_map.hit(p.x,p.y,1);
  lidar_map.ownFootprint(ego,1);auto lidar_grid=lidar_map.snapshot(ego,1);
  check(lidar_grid.footprint(ego,c),"270 degree scan cannot bootstrap observed footprint");
  const auto lidar_ref=reference(lidar_grid,ego,c);
  check(lidar_ref.size()>2,"270 degree scan cannot bootstrap reference");
  auto lidar_path=solve(ego,0,lidar_ref,obstacles(lidar_grid,ego,4),c);
  check(lidar_path.success,"scan-derived solve failed");
  const bool scan_valid=validatePath(lidar_grid,lidar_path.states,c,reason);
  if(!scan_valid){std::cerr<<reason<<" reference end "<<lidar_ref.back().x<<","<<lidar_ref.back().y<<'\n';
    for(int i=0;i<4;++i)std::cerr<<i<<": "<<lidar_path.states[i].x<<","<<lidar_path.states[i].y<<","<<lidar_path.states[i].yaw<<'\n';}
  check(scan_valid,"scan-derived swept path invalid");
  // Receding-horizon execution around a left corner, with perfect model actuation.
  // This tests repeated replanning/warm starts, not the ROS tracker or physical vehicle.
  auto corner_grid=grid();
  for(int i=0;i<static_cast<int>(corner_grid.cells.size());++i){const auto p=corner_grid.center(i);
    const bool horizontal=p.x<1.8 && std::abs(p.y)<0.8;
    const bool vertical=p.x>0.2 && p.x<1.8 && p.y>=0 && p.y<5;
    corner_grid.cells[i]=(horizontal || vertical)?0:100;}
  Pose moving{};double steering=0,peak_ms=0;std::vector<State>warm;
  for(int step=0;step<24;++step){
    const auto route=reference(corner_grid,moving,c,warm);
    auto next=solve(moving,steering,route,obstacles(corner_grid,moving,4),c,warm);
    if(!next.success)std::cerr<<"corner step "<<step<<": "<<next.reason<<'\n';
    check(next.success,"corner replanning failed");
    peak_ms=std::max(peak_ms,next.elapsed_ms);
    check(validatePath(corner_grid,next.states,c,reason),"corner swept path invalid");
    moving=next.states[1];steering=next.states[0].steering;warm=std::move(next.states);
  }
  check(moving.y>0.4,"receding planner did not turn into corridor");
  std::cout<<"24-step ideal-model corner: x="<<moving.x<<" y="<<moving.y<<" yaw="<<moving.yaw<<" peak_solve_ms="<<peak_ms<<'\n';
  std::cout<<"All core checks passed. ROS integration is not covered by this executable.\n";
  return EXIT_SUCCESS;
} catch(const std::exception&e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return EXIT_FAILURE;}
