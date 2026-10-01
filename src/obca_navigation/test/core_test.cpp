#include "obca_navigation/core.hpp"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <stdexcept>
namespace obca { double derivativeError(); }
void check(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
obca::Grid grid() {obca::Grid g;g.width=g.height=121;g.resolution=0.1;g.x0=g.y0=-6.05;g.cells.assign(g.width*g.height,0);return g;}
int main()try {
  using namespace obca;
  Config c;c.solve_seconds=5;c.validate();Pose ego{};
  check(std::abs(localStopBuffer(0.116,0.12)-0.0116)<1e-9,
    "short local recovery is classified as already arrived");
  check(localStopBuffer(2.0,0.12)==0.12,"long path stop buffer changed");
  check(localStopBuffer(0.0,0.12)==0.0,"zero-length path stop buffer invalid");
  std::vector<State> preview(3);preview[1].x=1.0;preview[2].x=2.0;
  Config fast=c;fast.max_speed=2.0;fast.max_lateral_accel=1.0;
  check(cornerPreviewSpeed(preview,0,fast)==2.0,"straight preview needlessly limits speed");
  preview[2].steering=std::atan(fast.wheelbase*2.0);
  check(std::abs(cornerPreviewSpeed(preview,2,fast)-std::sqrt(0.5))<1e-9,
    "corner preview misses lateral acceleration bound");
  check(cornerPreviewSpeed(preview,1.5,fast)<1.5,"preview did not brake before future corner");
  check(cornerPreviewSpeed(preview,0,fast)>cornerPreviewSpeed(preview,1.5,fast),
    "preview braking does not account for corner distance");
  check(reachableDistance(0,fast)<fast.max_speed*fast.dt*fast.horizon,
    "reachable distance ignores acceleration and terminal braking");
  std::vector<Pose> circle;
  for(int i=0;i<120;++i){const double a=i*2*std::acos(-1.0)/120;circle.push_back({3*std::cos(a),3*std::sin(a),a+std::acos(-1.0)/2,0.4});}
  Raceline line(circle);check(line.reset(circle[0],0.3,1.0),"raceline initialization failed");
  for(int i=1;i<=240;++i){const auto route=line.reference(circle[i%120],1.5,0.1,0.3,1.0,1.0,0.2);
    check(route.size()>2,"raceline wrap lost tracking");}
  check(line.progress()>1.9*line.length(),"raceline lap progress did not unwrap");
  Pose reverse_pose=circle[0];reverse_pose.yaw+=std::acos(-1.0);
  check(line.reset(reverse_pose,0.3,1.0),"initial heading did not reverse raceline");
  auto reversed=line.reference(reverse_pose,1,0.1,0.3,1.0,1.0,0.2);
  check(reversed.size()>2 && reversed[1].y<reverse_pose.y,"reversed raceline points wrong way");
  check(line.reference(circle[60],1,0.1,0.3,1.0,1.0,0.2).empty(),"raceline projection jumped outside continuity window");
  {std::ofstream csv("raceline_test_input.csv");csv<<"id,x_m,y_m,psi_rad,vx_mps\n0,0,0,0,0.2\n1,2,0,1.57,0.2\n2,2,2,3.14,0.2\n";}
  check(Raceline::loadCsv("raceline_test_input.csv").points().size()==3,"generator CSV not loaded");
  std::remove("raceline_test_input.csv");
  auto g=grid();
  check(derivativeError()<1e-6,"analytic gradient/Jacobian/Hessian mismatch");
  check(g.footprint(ego,c),"empty-space footprint");
  g.cells[g.index(0.3,0.0)]=-1;check(!g.footprint(ego,c),"unknown space must block body");
  g=grid();g.cells[g.index(0.3,0)]=100;check(!g.footprint(ego,c),"occupied body accepted");
  g=grid();
  const auto ref=reference(g,ego,c);check(ref.size()>2,"forward reference missing");check(ref.back().x>1,"reference must advance");
  // An off-centre start must converge toward the observed corridor centre,
  // with symmetric behaviour on either wall and with unknown boundaries too.
  Config centered=c;centered.reference_clearance=0.35;
  centered.reference_wall_weight=2.0;centered.goal_clearance_weight=1.5;
  for(int boundary : {100,-1})for(double offset : {-0.4,0.4}) {
    auto lane=grid();
    for(int i=0;i<static_cast<int>(lane.cells.size());++i)
      if(std::abs(lane.center(i).y)>=1.1)lane.cells[i]=boundary;
    const Pose start{0,offset,0,0};
    const auto baseline=reference(lane,start,c), middle=reference(lane,start,centered);
    check(!baseline.empty(),"baseline corridor reference missing");
    check(middle.size()>2 && middle.back().x>1.5,"centering lost forward progress");
    std::cout<<"centering: "<<offset<<" -> "<<middle.back().y<<" baseline "<<baseline.back().y<<'\n';
    check(std::abs(middle.back().y)<0.16,"local reference did not converge to corridor centre");
    check(std::abs(middle.back().y)<std::abs(baseline.back().y),"centering did not improve baseline");
    double arc=0;
    for(std::size_t i=1;i<middle.size();++i){arc+=distance(middle[i-1],middle[i]);
      check(lane.cells[lane.index(middle[i].x,middle[i].y)]==0,"centering entered blocked space");}
    check(arc<=centered.reference_distance+1e-8,"clearance cost changed geometric horizon");
    if(boundary==100) {
      Config tracking=centered;tracking.position_weight=16.0;
      const auto optimized=solve(start,0,middle,obstacles(lane,start,4),tracking);
      std::string why;
      check(optimized.success,"off-centre corridor optimization failed");
      check(validatePath(lane,optimized.states,tracking,why),"centered swept body violates corridor");
      check(std::abs(optimized.states.back().y)<0.25,"optimized path lost reference centering");
    }
  }
  auto solved=solve(ego,0,ref,{},c);
  auto slow_ref=ref;for(auto&p:slow_ref)p.v=0.2;
  auto slow=solve(ego,0,slow_ref,{},c,{},false,true);
  check(slow.success,"raceline speed profile solve failed");
  for(const auto&p:slow.states)check(p.v<=0.2+c.validation_tolerance,"raceline speed profile ignored");
  std::cout<<"straight: "<<solved.reason<<" "<<solved.elapsed_ms<<" ms\n";
  check(solved.success,"empty-space solver failed");
  check(solved.states.back().x>0.5,"solver did not progress");
  for(const auto &state:solved.states)check(distance(state,ego)<=reachableDistance(ego.v,c),
    "reachable obstacle bound excludes a solved body position");
  std::string reason;check(validatePath(g,solved.states,c,reason),"straight path invalid");
  check(std::abs(solved.states.back().v)<1e-6,"terminal stop constraint missing");
  const std::vector<Box> walls{{-1,0.7,4,0.9},{-1,-0.9,4,-0.7}};
  auto corridor=solve(ego,0,ref,walls,c);
  std::cout<<"corridor: "<<corridor.reason<<" "<<corridor.elapsed_ms<<" ms\n";
  check(corridor.success,"OBCA corridor solve failed");
  check(corridor.collision_pairs<static_cast<int>((c.horizon+1)*walls.size()),"unreachable knot constraints were retained");
  const auto far=solve(ego,0,ref,{{10,10,11,11}},c);
  check(far.success && far.collision_pairs==0,"unreachable obstacle was not excluded");
  for(const auto&p:corridor.states)for(const auto&b:walls)check(!overlap(p,c.front,c.rear,c.half_width,b),"OBCA intersects wall");
  Pose tracked=corridor.states[2];tracked.y+=0.06;tracked.yaw+=0.04;
  const auto replan=solve(tracked,corridor.states[2].steering,
    reference(g,tracked,c,corridor.states),walls,c,corridor.states);
  check(replan.success && validatePath(g,replan.states,c,reason),
    "warm replan after tracking displacement is invalid");
  check(distance(replan.states.front(),tracked)<1e-8 &&
    std::abs(angle(replan.states.front().yaw-tracked.yaw))<1e-8,
    "warm replan is not rooted in measured vehicle state");
  // A static obstacle on the original centreline: reference search must go around it,
  // and the optimized vehicle rectangle must clear it, not just its centre point.
  auto detour_grid=grid();
  for(int i=0;i<static_cast<int>(detour_grid.cells.size());++i){const auto p=detour_grid.center(i);
    if(p.x>=1.1 && p.x<=1.4 && std::abs(p.y)<=0.15)detour_grid.cells[i]=100;}
  const auto detour_ref=reference(detour_grid,ego,c);
  const auto startup_blocked=straightReference(detour_grid,ego,c);
  check(!startup_blocked.empty() && startup_blocked.back().x<1.0-c.front,
    "startup straight reference crossed an observed obstacle");
  const auto detour_boxes=obstacles(detour_grid,ego,4);
  auto detour=solve(ego,0,detour_ref,detour_boxes,c);
  std::cout<<"detour: "<<detour.reason<<" "<<detour.elapsed_ms<<" ms\n";
  check(detour.success,"static obstacle detour solve failed");
  check(validatePath(detour_grid,detour.states,c,reason),"detour failed independent collision validation");
  Config buffered=c;buffered.margin=0.12;buffered.reference_clearance=0.35;
  auto buffered_detour=solve(ego,0,reference(detour_grid,ego,buffered),detour_boxes,buffered);
  if(!buffered_detour.success)std::cerr<<"buffered detour: "<<buffered_detour.reason<<'\n';
  check(buffered_detour.success,"larger operational wall clearance prevents feasible detour");
  check(validatePath(detour_grid,buffered_detour.states,buffered,reason),"buffered detour violates wall clearance");
  // A measured pose can safely move away from a rear obstacle even when its
  // clearance is below the extra reserve required at future trajectory knots.
  auto recoverable=solve(Pose{0,0,0,0.3},0,ref,{{-0.5,-0.3,-0.26,0.3}},c);
  check(recoverable.success,"safe measured pose incorrectly requires future swept-step reserve");
  auto blocked=solve(ego,0,ref,{{-0.2,-0.3,0.5,0.3}},c);
  check(!blocked.success && blocked.states.empty(),"infeasible initial collision accepted");
  std::vector<Box> excess(c.max_obstacles+1);const auto overflow=solve(ego,0,ref,excess,c);
  check(!overflow.success && overflow.reason.find("obstacle budget exceeded: count=")==0,"obstacle overflow silently truncated or misreported");
  check(solve(ego,0,{}, {},c).reason.find("reference missing:")==0,"missing reference misreported as obstacle overflow");
  Config deadline=c;deadline.solve_seconds=1e-9;check(!solve(ego,0,ref,walls,deadline).success,"deadline ignored");
  auto unknown=grid();std::fill(unknown.cells.begin(),unknown.cells.end(),-1);
  check(reference(unknown,ego,c).empty(),"unknown grid planned through");
  check(reference(unknown,ego,centered).empty(),"centering treated unknown space as free");
  check(!validatePath(unknown,solved.states,c,reason),"unknown swept path accepted");
  unknown.has_known_body=true;unknown.known_body=ego;
  check(unknown.footprint(ego,c),"exact physical body is not recognized in blind spot");
  check(reference(unknown,ego,c).empty(),"bootstrap connector entered unknown space");
  check(straightReference(unknown,ego,c).empty(),"startup straight reference entered unknown space");
  Pose outside=ego;outside.y=0.02;
  check(!unknown.footprint(outside,c),"self-footprint exemption leaked into unknown space");
  // At rest, even a tiny initial turn sweeps the rear corner outside the known
  // physical body into the LiDAR blind spot. Recover only by a validated solve.
  auto blind=grid();blind.has_known_body=true;blind.known_body=ego;
  for(int i=0;i<static_cast<int>(blind.cells.size());++i)
    if(blind.center(i).x<0.15)blind.cells[i]=-1;
  const std::vector<Pose> turning{{0,0,0,0},{0.5,0,0.07,0},{2,0.3,0.07,0}};
  auto rejected_turn=solve(ego,0,turning,{},buffered);
  check(rejected_turn.success && !validatePath(blind,rejected_turn.states,buffered,reason),
    "blind-spot turning failure was not reproduced");
  auto observed_recovery=solveObserved(blind,ego,0,turning,{},buffered);
  check(observed_recovery.success && observed_recovery.reason.find("recovery=")!=std::string::npos,
    "stationary blind-spot recovery missing");
  check(validatePath(blind,observed_recovery.states,buffered,reason),"recovery crosses unknown space");
  check(observed_recovery.states.back().x>0.2,"recovery did not advance");
  for(const auto &state:observed_recovery.states)
    check(std::abs(state.yaw)<1e-8 && state.v<=0.3+buffered.validation_tolerance,
      "recovery is not a low-speed straight path");
  ObservationRecovery phase;
  auto phased_start=solveObserved(blind,ego,0,turning,{},buffered,{},&phase);
  check(phased_start.success && phase.active,"blind recovery phase not armed");
  Pose creeping{0.05,0,0,0.1};auto continuing_grid=blind;continuing_grid.known_body=creeping;
  auto continuing=solveObserved(continuing_grid,creeping,0,turning,{},buffered,{},&phase);
  check(continuing.success && phase.active && continuing.reason.find("recovery=")!=std::string::npos,
    "recovery oscillates into an unsafe turn while moving");
  check(validatePath(continuing_grid,continuing.states,buffered,reason),"continued recovery unsafe");
  Pose clear_start{0.75,0,0,0.1};auto released=solveObserved(grid(),clear_start,0,
    reference(grid(),clear_start,buffered),{},buffered,{},&phase);
  check(released.success && !phase.active,"recovery did not release after observed progress");
  phase.active=true;phase.origin=ego;
  const auto blocked_recovery=solveObserved(unknown,ego,0,turning,{},buffered,{},&phase);
  check(!blocked_recovery.success && !phase.active,"blocked recovery remains latched");
  Pose almost_stopped=ego;almost_stopped.v=0.005;
  auto creep_recovery=solveObserved(blind,almost_stopped,0,turning,{},buffered);
  check(creep_recovery.success && validatePath(blind,creep_recovery.states,buffered,reason),
    "gym residual stop speed permanently prevents recovery");
  Pose blind_moving=ego;blind_moving.v=0.1;
  check(!solveObserved(blind,blind_moving,0,turning,{},buffered).success,"blind recovery allowed while moving");
  check(!solveObserved(unknown,ego,0,turning,{},buffered).success,"recovery filled unknown space");
  LocalMap map(c);map.ray(0,0,2,0,1);map.hit(2,0,1);
  auto observed=map.snapshot(ego,1);check(observed.cells[observed.index(1,0)]==0,"ray free space missing");
  check(observed.cells[observed.index(2,0)]==100,"ray endpoint missing");
  observed=map.snapshot(ego,1+c.map_ttl+0.01);check(observed.cells[observed.index(1,0)]==-1,"expired free space retained");
  map.hit(1,0,10);map.clear();observed=map.snapshot(ego,10);check(observed.cells[observed.index(1,0)]==-1,"reset retained old map");
  Config memory_config=c;memory_config.retain_observations=true;
  LocalMap memory(memory_config);
  memory.ray(0,0,2,0,1);memory.hit(2,0,1);
  auto remembered=memory.snapshot(ego,20);
  check(remembered.cells[remembered.index(1,0)]==0,"SLAM free observation expired in blind spot");
  check(remembered.cells[remembered.index(2,0)]==100,"SLAM occupied observation expired");
  check(remembered.cells[remembered.index(1,1)]==-1,"SLAM memory invents unobserved free space");
  memory.hit(1,0,21);remembered=memory.snapshot(ego,21);
  check(remembered.cells[remembered.index(1,0)]==100,"new obstacle ignored by SLAM memory");
  memory.ray(0,0,2,0,22);memory.hit(2,0,22);remembered=memory.snapshot(ego,22);
  check(remembered.cells[remembered.index(1,0)]==0,"new free observation ignored by SLAM memory");
  memory.snapshot(Pose{20,0,0,0},23);remembered=memory.snapshot(ego,24);
  check(remembered.cells[remembered.index(1,0)]==-1,"SLAM memory exceeds local radius bound");
  memory.ray(0,0,2,0,25);memory.clear();remembered=memory.snapshot(ego,26);
  check(remembered.cells[remembered.index(1,0)]==-1,"manual reset retains SLAM observations");
  memory.ray(0,0,2,0,30);remembered=memory.snapshot(ego,29);
  check(remembered.cells[remembered.index(1,0)]==-1,"SLAM memory retains observations from future clock");
  // 180-degree forward LiDAR: a scan at an earlier pose observes today's rear.
  // At rest the current scan cannot refresh it, but SLAM memory must retain it.
  LocalMap rolling_scan(c),accumulated_scan(memory_config);
  auto forward_scan=[](LocalMap &target,double x,double stamp) {
    std::vector<Pose> endpoints;
    for(int i=0;i<=1080;++i) {
      const double a=(-90.0+i/6.0)*std::acos(-1.0)/180;
      const double dx=std::cos(a),dy=std::sin(a),ox=x+0.275;
      double range=6.0;
      if(std::abs(dy)>1e-9)range=std::min(range,0.8/std::abs(dy));
      if(dx>1e-9)range=std::min(range,(4.0-ox)/dx);
      Pose hit{ox+range*dx,range*dy,0,0};
      target.ray(ox,0,hit.x,hit.y,stamp);endpoints.push_back(hit);
    }
    for(const auto &hit:endpoints)target.hit(hit.x,hit.y,stamp);
    target.ownFootprint(Pose{x,0,0,0},stamp);
  };
  for(auto *target:{&rolling_scan,&accumulated_scan}) {
    forward_scan(*target,-1.0,1.0);forward_scan(*target,0,10.0);
  }
  const auto rolling_grid=rolling_scan.snapshot(ego,10.0);
  const auto accumulated_grid=accumulated_scan.snapshot(ego,10.0);
  Pose initial_turn{0.04,0,0.01,0};
  check(!rolling_grid.footprint(initial_turn,c),"180-degree blind-spot failure not reproduced");
  check(accumulated_grid.footprint(initial_turn,c),"observed SLAM history does not permit initial turn");
  check(reference(accumulated_grid,ego,memory_config).size()>2,"observed SLAM history cannot seed local goals");
  const auto remembered_turn=solveObserved(accumulated_grid,ego,0,turning,
    obstacles(accumulated_grid,ego,reachableDistance(ego.v,memory_config)+0.7),memory_config);
  check(remembered_turn.success && validatePath(accumulated_grid,remembered_turn.states,memory_config,reason),
    "observed SLAM history cannot produce a validated turn");
  check(remembered_turn.reason.find("recovery=")==std::string::npos,
    "observed rear still forces straight-only recovery");
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
  // The same observed corridor must bootstrap away from a grid-aligned origin too.
  for(int heading=0;heading<24;++heading) {
    Pose start{0.037,-0.023,heading*std::acos(-1.0)/12,0};
    auto world=[&](double x,double y){return Pose{start.x+std::cos(start.yaw)*x-std::sin(start.yaw)*y,
      start.y+std::sin(start.yaw)*x+std::cos(start.yaw)*y,0,0};};
    LocalMap rotated(c);std::vector<Pose> endpoints;const auto sensor=world(0.27,0);
    for(int beam=0;beam<=1080;++beam) {
      const double a=(-135.0+beam*0.25)*std::acos(-1.0)/180,dx=std::cos(a),dy=std::sin(a);
      double range=6;
      if(std::abs(dy)>1e-9)range=std::min(range,0.8/std::abs(dy));
      if(dx>1e-9)range=std::min(range,(4-0.27)/dx);
      const auto hit=world(0.27+range*dx,range*dy);
      rotated.ray(sensor.x,sensor.y,hit.x,hit.y,1);endpoints.push_back(hit);
    }
    for(const auto&p:endpoints)rotated.hit(p.x,p.y,1);
    rotated.ownFootprint(start,1);const auto observed_grid=rotated.snapshot(start,1);
    check(observed_grid.footprint(start,c),"rotated scan cannot bootstrap footprint");
    const auto route=reference(observed_grid,start,c);
    if(route.size()<2)std::cerr<<"missing rotated reference at heading "<<heading<<'\n';
    check(route.size()>2,"rotated scan cannot bootstrap reference");
    const auto straight=straightReference(observed_grid,start,c);
    check(straight.size()>2,"observed startup straight reference missing");
    Config startup_config=c;startup_config.max_obstacles=128; // Isolate geometry from the independent budget test.
    const auto startup=solve(start,0,straight,obstacles(observed_grid,start,4),startup_config,{},true);
    if(!startup.success)std::cerr<<"startup heading "<<heading<<": "<<startup.reason<<'\n';
    check(startup.success,"zero-steering startup solve failed");
    for(const auto&p:startup.states)check(std::abs(p.steering)<1e-8 && std::abs(p.yaw-start.yaw)<1e-8,"startup steering constraint missing");
    check(validatePath(observed_grid,startup.states,c,reason),"startup swept footprint entered unknown space");
  }
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
