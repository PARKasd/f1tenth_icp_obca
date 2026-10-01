#include "obca_navigation/core.hpp"
#include <algorithm>
#include <chrono>
#include <limits>
#include <queue>
#include <stdexcept>

namespace obca {
namespace {
using Polygon=std::vector<Pose>;
Polygon rectangle(const Pose&p,double front,double rear,double hw) {
  Polygon out;
  for(const auto&v:std::vector<std::pair<double,double>>{{front,hw},{-rear,hw},{-rear,-hw},{front,-hw}})
    out.push_back({p.x+std::cos(p.yaw)*v.first-std::sin(p.yaw)*v.second,
      p.y+std::sin(p.yaw)*v.first+std::cos(p.yaw)*v.second,0,0});
  return out;
}
double cross(const Pose&a,const Pose&b,const Pose&c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
Polygon hull(Polygon points) {
  std::sort(points.begin(),points.end(),[](const Pose&a,const Pose&b){return a.x<b.x || (a.x==b.x && a.y<b.y);});
  points.erase(std::unique(points.begin(),points.end(),[](const Pose&a,const Pose&b){return a.x==b.x && a.y==b.y;}),points.end());
  if(points.size()<3)return points;
  Polygon h;
  for(const auto&p:points){while(h.size()>1 && cross(h[h.size()-2],h.back(),p)<=0)h.pop_back();h.push_back(p);}
  const auto lower=h.size();
  for(auto it=points.rbegin()+1;it!=points.rend();++it){while(h.size()>lower && cross(h[h.size()-2],h.back(),*it)<=0)h.pop_back();h.push_back(*it);}
  h.pop_back();return h;
}
Polygon expand(const Polygon&poly,double padding) {
  Polygon points;
  for(const auto&p:poly)for(double x:{-padding,padding})for(double y:{-padding,padding})points.push_back({p.x+x,p.y+y,0,0});
  return hull(std::move(points));
}
Polygon clip(Polygon poly,const Box&b) {
  for(int axis=0;axis<4;++axis) {
    auto signed_distance=[&](const Pose&p){switch(axis){case 0:return p.x-b.xmin;case 1:return b.xmax-p.x;case 2:return p.y-b.ymin;default:return b.ymax-p.y;}};
    Polygon next;if(poly.empty())break;
    Pose previous=poly.back();double pd=signed_distance(previous);
    for(const auto&p:poly){const double d=signed_distance(p);
      if((d>=0)!=(pd>=0)){const double f=pd/(pd-d);next.push_back({previous.x+f*(p.x-previous.x),previous.y+f*(p.y-previous.y),0,0});}
      if(d>=0)next.push_back(p);
      previous=p;pd=d;
    }
    poly=std::move(next);
  }
  return poly;
}
bool clearPolygons(const Grid&g,const Polygon&body,const Polygon&guard,const Config&c) {
  if(guard.empty())return false;
  Box bounds{guard[0].x,guard[0].y,guard[0].x,guard[0].y};
  for(const auto&p:guard){bounds.xmin=std::min(bounds.xmin,p.x);bounds.ymin=std::min(bounds.ymin,p.y);bounds.xmax=std::max(bounds.xmax,p.x);bounds.ymax=std::max(bounds.ymax,p.y);}
  const int lo=g.index(bounds.xmin,bounds.ymin),hi=g.index(bounds.xmax,bounds.ymax);
  if(lo<0 || hi<0)return false;
  for(int y=lo/g.width;y<=hi/g.width;++y)for(int x=lo%g.width;x<=hi%g.width;++x) {
    const int value=g.cells[y*g.width+x];if(value==0)continue;
    const Box cell{g.x0+x*g.resolution,g.y0+y*g.resolution,g.x0+(x+1)*g.resolution,g.y0+(y+1)*g.resolution};
    const auto intersection=clip(value>0?guard:body,cell);
    double twice_area=0;
    for(std::size_t i=0;i<intersection.size();++i){const auto&a=intersection[i];const auto&b=intersection[(i+1)%intersection.size()];twice_area+=a.x*b.y-a.y*b.x;}
    if(std::abs(twice_area)<1e-12)continue;
    if(value>0 || !g.has_known_body)return false;
    // Do not clear an entire unobserved cell merely because the car touches it.
    // Only its exact intersection with the current physical body is known free.
    for(const auto&p:intersection){const double dx=p.x-g.known_body.x,dy=p.y-g.known_body.y;
      const double lx=std::cos(g.known_body.yaw)*dx+std::sin(g.known_body.yaw)*dy;
      const double ly=-std::sin(g.known_body.yaw)*dx+std::cos(g.known_body.yaw)*dy;
      if(lx>c.front+1e-6 || lx< -c.rear-1e-6 || std::abs(ly)>c.half_width+1e-6)return false;
    }
  }
  return true;
}
} // namespace
double reachableDistance(double speed,const Config &c) {
  double reach=0;
  for(int i=1;i<=c.horizon;++i)
    reach+=c.dt*std::min({c.max_speed,speed+(i-1)*c.dt*c.max_accel,
      (c.horizon-i+1)*c.dt*c.max_decel});
  return reach+c.horizon*(std::sqrt(2.0)+c.horizon*c.dt)*c.validation_tolerance;
}
double cornerPreviewSpeed(const std::vector<State> &path,double progress,const Config &c) {
  double limit=c.max_speed,arc=0;
  for(std::size_t i=0;i<path.size();++i) {
    if(i)arc+=distance(path[i-1],path[i]);
    if(arc<progress)continue;
    const double curvature=std::abs(std::tan(path[i].steering)/c.wheelbase);
    if(curvature<1e-9)continue;
    const double turn_speed_squared=c.max_lateral_accel/curvature;
    limit=std::min(limit,std::sqrt(turn_speed_squared+2*c.max_decel*(arc-progress)));
  }
  return limit;
}
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double distance(const Pose &a, const Pose &b) { return std::hypot(a.x-b.x, a.y-b.y); }
void Config::validate() const {
  const double positive[] = {dt,wheelbase,front,rear,half_width,margin,max_speed,max_steering,
    max_steering_rate,max_accel,max_decel,max_lateral_accel,solve_seconds,tolerance,
    validation_tolerance,position_weight,heading_weight,speed_weight,steering_weight,
    acceleration_weight,smooth_weight,grid_resolution,map_radius,map_ttl,
    reference_distance,reference_clearance,validation_step,seed_lookahead,recovery_stationary_speed,goal_clearance_target,goal_continuation_distance};
  for (double v : positive) if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("nonpositive/nonfinite configuration");
  for(double v : {reference_wall_weight,goal_clearance_weight,goal_progress_weight,
      goal_route_clearance_weight,goal_turn_weight,goal_continuation_weight,
      goal_forward_weight,goal_lateral_weight,goal_path_weight,goal_continuity_weight})
    if(!std::isfinite(v) || v<0)throw std::invalid_argument("invalid reference/goal weight");
  if (horizon < 4 || horizon > 80 || max_obstacles < 1 || max_obstacles > 512 ||
      max_iterations < 1 || max_steering >= 1.4 || map_radius/grid_resolution > 250 ||
      reference_distance >= map_radius || reference_clearance < half_width ||
      recovery_stationary_speed>max_speed || validation_step > grid_resolution/2 || max_speed/max_decel >= horizon*dt)
    throw std::invalid_argument("inconsistent planning configuration");
}
bool overlap(const Pose &p, double front, double rear, double hw, const Box &b) {
  // Separating axis test: oriented vehicle rectangle versus axis-aligned grid cell.
  const double c=std::cos(p.yaw), s=std::sin(p.yaw), hl=(front+rear)/2;
  const double cx=p.x+c*(front-rear)/2, cy=p.y+s*(front-rear)/2;
  const double bx=(b.xmin+b.xmax)/2, by=(b.ymin+b.ymax)/2;
  const double ex=(b.xmax-b.xmin)/2, ey=(b.ymax-b.ymin)/2;
  const double dx=bx-cx, dy=by-cy;
  return std::abs(dx)<=ex+hl*std::abs(c)+hw*std::abs(s) &&
    std::abs(dy)<=ey+hl*std::abs(s)+hw*std::abs(c) &&
    std::abs(c*dx+s*dy)<=hl+ex*std::abs(c)+ey*std::abs(s) &&
    std::abs(-s*dx+c*dy)<=hw+ex*std::abs(s)+ey*std::abs(c);
}
int Grid::index(double x, double y) const {
  const int ix=static_cast<int>(std::floor((x-x0)/resolution));
  const int iy=static_cast<int>(std::floor((y-y0)/resolution));
  return ix>=0 && iy>=0 && ix<width && iy<height ? iy*width+ix : -1;
}
Pose Grid::center(int i) const { return {x0+(i%width+0.5)*resolution,y0+(i/width+0.5)*resolution,0,0}; }
bool Grid::footprint(const Pose &p, const Config &c) const {
  const auto body=rectangle(p,c.front,c.rear,c.half_width);
  return clearPolygons(*this,body,expand(body,c.margin),c);
}
void LocalMap::ray(double ox,double oy,double ex,double ey,double t) {
  const double r=c_.grid_resolution;
  int x=static_cast<int>(std::floor(ox/r)), y=static_cast<int>(std::floor(oy/r));
  const int tx=static_cast<int>(std::floor(ex/r)), ty=static_cast<int>(std::floor(ey/r));
  const int dx=std::abs(tx-x), dy=-std::abs(ty-y), sx=x<tx?1:-1, sy=y<ty?1:-1;
  int error=dx+dy;
  while(true) {
    cells_[{x,y}]={0,t};
    if(x==tx && y==ty) break;
    const int twice=2*error;
    if(twice>=dy) {error+=dy;x+=sx;}
    if(twice<=dx) {error+=dx;y+=sy;}
  }
}
void LocalMap::hit(double x,double y,double t) {
  cells_[{static_cast<int>(std::floor(x/c_.grid_resolution)),static_cast<int>(std::floor(y/c_.grid_resolution))}]={100,t};
}
void LocalMap::ownFootprint(const Pose &p,double t) {
  const double r=c_.grid_resolution, radius=std::hypot(std::max(c_.front,c_.rear),c_.half_width);
  for(int y=static_cast<int>(std::floor((p.y-radius)/r));y<=static_cast<int>(std::floor((p.y+radius)/r));++y)
    for(int x=static_cast<int>(std::floor((p.x-radius)/r));x<=static_cast<int>(std::floor((p.x+radius)/r));++x) {
      // Only completely enclosed cells are known free from the physical vehicle footprint.
      bool inside=true;
      for(int v=0;v<2;++v) for(int u=0;u<2;++u) {
        const double dx=(x+u)*r-p.x,dy=(y+v)*r-p.y;
        const double lx=std::cos(p.yaw)*dx+std::sin(p.yaw)*dy;
        const double ly=-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy;
        inside=inside && lx>=-c_.rear && lx<=c_.front && std::abs(ly)<=c_.half_width;
      }
      if(inside && cells_.find({x,y})==cells_.end()) cells_[{x,y}]={0,t};
    }
}
Grid LocalMap::snapshot(const Pose &p,double t) {
  const double r=c_.grid_resolution;
  for(auto it=cells_.begin();it!=cells_.end();) {
    const double dx=(it->first.first+0.5)*r-p.x,dy=(it->first.second+0.5)*r-p.y;
    if(t<it->second.time || t-it->second.time>c_.map_ttl || std::hypot(dx,dy)>c_.map_radius)
      it=cells_.erase(it);
    else ++it;
  }
  const int half=static_cast<int>(std::ceil(c_.map_radius/r));
  Grid g;g.width=g.height=2*half+1;g.resolution=r;
  g.known_body=p;g.has_known_body=true;
  const int x0=static_cast<int>(std::floor(p.x/r))-half,y0=static_cast<int>(std::floor(p.y/r))-half;
  g.x0=x0*r;g.y0=y0*r;g.cells.assign(g.width*g.height,-1);
  for(const auto &entry:cells_) {
    const int x=entry.first.first-x0,y=entry.first.second-y0;
    if(x>=0 && y>=0 && x<g.width && y<g.height) g.cells[y*g.width+x]=entry.second.value;
  }
  return g;
}
static std::vector<Pose> referenceSearch(const Grid &g,const Pose &ego,const Config &c,
    const std::vector<State> &previous,GoalEvaluation *evaluation,double prefix=0,
    const Pose *root=nullptr) {
  if(evaluation)*evaluation={};
  const Pose &origin_pose=root?*root:ego;
  const double budget=c.reference_distance-prefix;
  const int start=g.index(ego.x,ego.y);
  if(start<0 || g.cells[start]!=0) return {};
  std::vector<double> costs(g.cells.size(),std::numeric_limits<double>::infinity());
  std::vector<int> parents(g.cells.size(),-1), traversable(g.cells.size(),-1);
  std::vector<double> lengths(g.cells.size(),0.0), clearance(g.cells.size(),0.0);
  {
    // Eight-neighbour unit-distance transform is a conservative distance estimate.
    // Unknown cells and map edges are boundaries too; they never become free.
    std::vector<int> steps(g.cells.size(),-1);std::queue<int> wave;
    for(int i=0;i<static_cast<int>(g.cells.size());++i)
      if(g.cells[i]!=0 || i%g.width==0 || i%g.width==g.width-1 || i/g.width==0 || i/g.width==g.height-1) {
        steps[i]=0;wave.push(i);
      }
    while(!wave.empty()) {
      const int i=wave.front();wave.pop();const int ix=i%g.width,iy=i/g.width;
      for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x) {
        const int nx=ix+x,ny=iy+y;
        if(nx<0 || nx>=g.width || ny<0 || ny>=g.height)continue;
        const int j=ny*g.width+nx;
        if(steps[j]<0){steps[j]=steps[i]+1;wave.push(j);}
      }
    }
    for(std::size_t i=0;i<steps.size();++i)
      clearance[i]=std::max(0.0,(steps[i]-0.5)*g.resolution);
  }
  auto free=[&](int i) {
    if(i<0) return false;
    if(traversable[i]>=0) return traversable[i]!=0;
    const Pose p=g.center(i); const int radius=static_cast<int>(std::ceil(c.reference_clearance/g.resolution));
    auto known=[&](int j){
      if(j<0 || g.cells[j]>0)return false;
      if(g.cells[j]==0)return true;
      if(!g.has_known_body)return false;
      const auto point=g.center(j);const double dx=point.x-g.known_body.x,dy=point.y-g.known_body.y;
      const double x=std::cos(g.known_body.yaw)*dx+std::sin(g.known_body.yaw)*dy;
      const double y=-std::sin(g.known_body.yaw)*dx+std::cos(g.known_body.yaw)*dy;
      return x>=-c.rear && x<=c.front && std::abs(y)<=c.half_width;
    };
    bool ok=known(i);
    for(int y=-radius;y<=radius && ok;++y) for(int x=-radius;x<=radius && ok;++x) {
      if(std::hypot(x*g.resolution,y*g.resolution)>c.reference_clearance) continue;
      const int j=g.index(p.x+x*g.resolution,p.y+y*g.resolution);
      ok=known(j);
    }
    traversable[i]=ok?1:0;return ok;
  };
  using Item=std::pair<double,int>;
  std::priority_queue<Item,std::vector<Item>,std::greater<Item>> q;
  costs[start]=0;q.push({0,start});
  std::vector<double> minimum_clearance(g.cells.size(),std::numeric_limits<double>::infinity());
  std::vector<double> initial_heading(g.cells.size(),ego.yaw);
  const Pose lattice_origin=g.center(start);
  int best=start;GoalEvaluation selected;
  while(!q.empty()) {
    const auto [cost,i]=q.top();q.pop();if(cost>costs[i])continue;
    const Pose p=g.center(i);
    if(i!=start && lengths[i]+prefix>c.reference_clearance) {
      // Estimate the exit direction over a spatial window; scoring individual
      // 45-degree lattice edges would punish smooth corridors for grid aliasing.
      int behind=i;
      while(parents[behind]>=0 && lengths[i]-lengths[behind]<c.seed_lookahead)
        behind=parents[behind];
      const Pose tail=g.center(behind);
      Pose candidate{p.x+ego.x-lattice_origin.x,p.y+ego.y-lattice_origin.y,
        std::atan2(p.y-tail.y,p.x-tail.x),0};
      GoalFeatures features;
      features.observed_connected=free(i) && g.footprint(candidate,c);
      features.progress_m=lengths[i]+prefix;features.path_cost_m=cost+prefix;
      const double dx=candidate.x-origin_pose.x,dy=candidate.y-origin_pose.y;
      features.lateral_m=-std::sin(origin_pose.yaw)*dx+std::cos(origin_pose.yaw)*dy;
      features.goal_clearance_m=clearance[i];features.route_clearance_m=minimum_clearance[i];
      features.initial_heading_error_rad=angle(initial_heading[i]-origin_pose.yaw);
      features.heading_change_rad=angle(candidate.yaw-initial_heading[i]);
      features.has_previous=!previous.empty();
      if(features.has_previous)features.previous_goal_distance_m=distance(candidate,previous.back());
      if(features.observed_connected) {
        for(double advance=g.resolution;advance<=c.goal_continuation_distance+1e-9;advance+=g.resolution) {
          Pose ahead=candidate;ahead.x+=advance*std::cos(candidate.yaw);ahead.y+=advance*std::sin(candidate.yaw);
          if(!free(g.index(ahead.x,ahead.y)) || !g.footprint(ahead,c))break;
          features.continuation_m=advance;
        }
      }
      auto scored=scoreLocalGoal(features,c);scored.goal=candidate;
      if(scored.eligible && scored.total>selected.total){best=i;selected=scored;}
    }
    for(int y=-1;y<=1;++y) for(int x=-1;x<=1;++x) {
      if(x==0 && y==0) continue;
      const int j=g.index(p.x+x*g.resolution,p.y+y*g.resolution);
      if(!free(j))continue;
      const auto np=g.center(j);
      if(std::cos(ego.yaw)*(np.x-ego.x)+std::sin(ego.yaw)*(np.y-ego.y)<-c.reference_clearance)continue;
      if(x && y && (!free(g.index(p.x+x*g.resolution,p.y)) || !free(g.index(p.x,p.y+y*g.resolution))))continue;
      const double step=g.resolution*std::hypot(x,y);
      const double length=lengths[i]+step;
      const double proximity=c.reference_clearance/std::max(c.reference_clearance,clearance[j]);
      const double next=cost+step*(1+c.reference_wall_weight*proximity*proximity);
      if(length<=budget+1e-9 && next<costs[j]) {
        costs[j]=next;lengths[j]=length;parents[j]=i;
        minimum_clearance[j]=std::min(minimum_clearance[i],clearance[j]);
        if(length+prefix<=c.seed_lookahead+g.resolution)
          initial_heading[j]=std::atan2(np.y+ego.y-lattice_origin.y-origin_pose.y,
            np.x+ego.x-lattice_origin.x-origin_pose.x);
        else initial_heading[j]=initial_heading[i];
        q.push({next,j});
      }
    }
  }
  if(best==start)return {};
  if(evaluation)*evaluation=selected;
  std::vector<Pose> path;
  for(int i=best;i!=start;i=parents[i]) {if(i<0)return {};path.push_back(g.center(i));}
  path.push_back(ego);std::reverse(path.begin(),path.end());
  // Anchor the lattice to the actual ego position rather than introducing a half-cell
  // lateral step at every replan. The full optimized geometry is independently validated.
  const Pose origin=g.center(start);
  for(std::size_t i=1;i<path.size();++i){path[i].x+=ego.x-origin.x;path[i].y+=ego.y-origin.y;}
  for(std::size_t i=0;i+1<path.size();++i) path[i].yaw=std::atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x);
  path.back().yaw=path[path.size()-2].yaw;
  return path;
}
std::vector<Pose> reference(const Grid &g,const Pose &ego,const Config &c,
    const std::vector<State> &previous,GoalEvaluation *evaluation) {
  auto path=referenceSearch(g,ego,c,previous,evaluation);
  if(!path.empty() || !g.footprint(ego,c))return path;
  // Circular search clearance can touch the rear blind spot even though the actual
  // rectangle can move forward. Connect to the lattice only through verified free
  // swept body space; do not fill unknown cells or relax the collision validator.
  const double limit=std::min(c.reference_distance-c.reference_clearance,
    c.reference_clearance+std::max(c.front,c.rear)+g.resolution);
  for(double advance=g.resolution;advance<=limit;advance+=g.resolution) {
    Pose seed=ego;seed.x+=advance*std::cos(ego.yaw);seed.y+=advance*std::sin(ego.yaw);
    std::vector<State> connector(c.horizon+1);
    for(int i=0;i<=c.horizon;++i) {
      const double fraction=static_cast<double>(i)/c.horizon;
      connector[i].x=ego.x+fraction*(seed.x-ego.x);
      connector[i].y=ego.y+fraction*(seed.y-ego.y);connector[i].yaw=ego.yaw;
    }
    std::string reason;
    if(!validatePath(g,connector,c,reason))break;
    path=referenceSearch(g,seed,c,previous,evaluation,advance,&ego);
    if(!path.empty()){path.insert(path.begin(),ego);return path;}
  }
  return {};
}
std::vector<Box> obstacles(const Grid &g,const Pose &ego,double reach) {
  // Merge only adjacent occupied cells, never merge across a known-free gap.
  std::vector<Box> result;
  std::map<std::pair<int,int>,std::size_t> previous;
  for(int y=0;y<g.height;++y) {
    std::map<std::pair<int,int>,std::size_t> current;
    for(int x=0;x<g.width;) {
      const auto relevant=[&](int u) {const auto p=g.center(y*g.width+u);return g.cells[y*g.width+u]>0 &&
        std::abs(p.x-ego.x)<=reach+g.resolution && std::abs(p.y-ego.y)<=reach+g.resolution;};
      if(!relevant(x)){++x;continue;}
      const int begin=x;while(x<g.width && relevant(x))++x;
      const auto span=std::make_pair(begin,x);const auto found=previous.find(span);
      if(found!=previous.end()) {result[found->second].ymax=g.y0+(y+1)*g.resolution;current[span]=found->second;}
      else {current[span]=result.size();result.push_back({g.x0+begin*g.resolution,g.y0+y*g.resolution,g.x0+x*g.resolution,g.y0+(y+1)*g.resolution});}
    }
    previous=std::move(current);
  }
  return result;
}
double localStopBuffer(double path_length,double goal_tolerance) {
  return std::min(goal_tolerance,0.1*std::max(0.0,path_length));
}
std::vector<Pose> straightReference(const Grid &g,const Pose &ego,const Config &c) {
  if(!g.footprint(ego,c))return {};
  std::vector<Pose> path{ego};
  for(double d=g.resolution;d<=c.reference_distance;d+=g.resolution) {
    Pose next=ego;next.x+=d*std::cos(ego.yaw);next.y+=d*std::sin(ego.yaw);
    std::vector<State> segment(c.horizon+1);
    for(int i=0;i<=c.horizon;++i){const double f=static_cast<double>(i)/c.horizon;
      segment[i].x=path.back().x+f*(next.x-path.back().x);
      segment[i].y=path.back().y+f*(next.y-path.back().y);segment[i].yaw=ego.yaw;}
    std::string reason;if(!validatePath(g,segment,c,reason))break;
    path.push_back(next);
  }
  return path.size()>1?path:std::vector<Pose>{};
}
bool validatePath(const Grid &g,const std::vector<State> &path,const Config &c,std::string &reason) {
  if(path.size()!=static_cast<std::size_t>(c.horizon+1)){reason="invalid path length";return false;}
  const double radius=std::hypot(std::max(c.front,c.rear),c.half_width);
  for(std::size_t i=0;i<path.size();++i) {
    const State &p=path[i];
    if(!std::isfinite(p.x+p.y+p.yaw+p.v+p.steering+p.acceleration) || p.v< -c.validation_tolerance || p.v>c.max_speed+c.validation_tolerance) {reason="invalid state";return false;}
    const State &a=path[i?i-1:i];
    const double rotation=angle(p.yaw-a.yaw);
    const int steps=std::max(1,static_cast<int>(std::ceil((distance(a,p)+radius*std::abs(rotation))/c.validation_step)));
    for(int j=0;j<steps;++j) {
      const double f=static_cast<double>(j)/steps,f2=static_cast<double>(j+1)/steps;
      Pose first{a.x+f*(p.x-a.x),a.y+f*(p.y-a.y),a.yaw+f*rotation,0};
      Pose second{a.x+f2*(p.x-a.x),a.y+f2*(p.y-a.y),a.yaw+f2*rotation,0};
      auto points=rectangle(first,c.front,c.rear,c.half_width);const auto end=rectangle(second,c.front,c.rear,c.half_width);
      points.insert(points.end(),end.begin(),end.end());
      // Endpoint hull + rotational sagitta encloses the continuous interpolated body.
      const double sagitta=radius*(1-std::cos(std::abs(rotation)/(2*steps)));
      const auto swept=expand(hull(std::move(points)),sagitta);
      if(!clearPolygons(g,swept,expand(swept,c.margin),c)) {reason="swept footprint reaches occupied or unknown space at step "+std::to_string(i);return false;}
    }
  }
  if(std::abs(path.back().v)>c.validation_tolerance){reason="terminal speed is nonzero";return false;}
  reason="valid";return true;
}
Solution solveObserved(const Grid &grid,const Pose &ego,double steering,
    const std::vector<Pose> &ref,const std::vector<Box> &boxes,const Config &c,
    const std::vector<State> &warm,ObservationRecovery *phase) {
  const auto started=std::chrono::steady_clock::now();
  const auto elapsed=[&]{return std::chrono::duration<double>(
    std::chrono::steady_clock::now()-started).count();};
  const bool stationary=std::abs(ego.v)<=c.recovery_stationary_speed &&
    std::abs(steering)<=c.max_steering_rate*c.dt;
  auto recover=[&]() {
    Solution result;
    if(!stationary && !(phase && phase->active))return result;
    Config recovery=c;
    recovery.reference_distance=std::min(c.reference_distance,c.reference_clearance+c.front);
    recovery.max_speed=std::min(c.max_speed,0.3);
    recovery.solve_seconds=c.solve_seconds-elapsed();
    if(recovery.solve_seconds<=0)return result;
    const auto connector=straightReference(grid,ego,recovery);
    if(connector.size()<2)return result;
    result=solve(ego,steering,connector,boxes,recovery,phase && phase->active?warm:std::vector<State>{},true);
    std::string reason;
    if(!result.success){result.states.clear();return result;}
    if(!validatePath(grid,result.states,recovery,reason)) {
      result.success=false;result.reason="observation recovery blocked: "+reason;
      result.states.clear();return result;
    }
    double progress=0;
    for(std::size_t i=1;i<result.states.size();++i)
      progress+=distance(result.states[i-1],result.states[i]);
    if(progress<=c.validation_step) {
      result.success=false;result.states.clear();
      result.reason="observation recovery blocked: insufficient validated progress";
      return result;
    }
    result.elapsed_ms=elapsed()*1000.0;
    if(result.elapsed_ms>c.solve_seconds*1000){result.success=false;result.states.clear();return result;}
    result.reason="solved; recovery=observed_straight";
    if(phase && !phase->active){phase->active=true;phase->origin=ego;}
    return result;
  };
  if(phase && phase->active) {
    bool ready=distance(ego,phase->origin)>=c.rear+c.grid_resolution;
    for(double turn : {-0.01,0.01}) {
      Pose probe=ego;probe.x+=c.validation_step*std::cos(ego.yaw);
      probe.y+=c.validation_step*std::sin(ego.yaw);probe.yaw+=turn;
      ready=ready && grid.footprint(probe,c);
    }
    if(!ready) {
      auto straight=recover();
      if(!straight.success){phase->active=false;straight.states.clear();
        if(straight.reason.empty())straight.reason="observation recovery blocked: no validated straight continuation";}
      return straight;
    }
    phase->active=false;
  }
  // A tiny initial turn can sweep the rear corner out of the known current
  // body before enough forward space has been observed. Probe both turn signs
  // and try the validated connector first, avoiding a wasted full turning solve.
  if(stationary && ref.size()>=2) {
    bool blind=false;
    for(double turn : {-0.01,0.01}) {
      Pose probe=ego;probe.x+=c.validation_step*std::cos(ego.yaw);
      probe.y+=c.validation_step*std::sin(ego.yaw);probe.yaw+=turn;
      if(!grid.footprint(probe,c))blind=true;
    }
    if(blind){auto straight=recover();if(straight.success)return straight;}
  }
  Config remaining=c;remaining.solve_seconds=c.solve_seconds-elapsed();
  if(remaining.solve_seconds<=0){Solution result;result.reason="observation recovery deadline";return result;}
  auto solution=solve(ego,steering,ref,boxes,remaining,warm);
  solution.elapsed_ms=elapsed()*1000.0;
  if(!solution.success)return solution;
  std::string reason;
  if(validatePath(grid,solution.states,c,reason))return solution;
  solution.success=false;solution.states.clear();solution.reason=reason;
  // Never publish the rejected turn, relax unknown-space checks, or recover in motion.
  auto straight=recover();if(straight.success)return straight;
  solution.elapsed_ms=elapsed()*1000.0;
  return solution;
}
} // namespace obca
