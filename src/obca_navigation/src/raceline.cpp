#include "obca_navigation/core.hpp"
#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace obca {
Raceline::Raceline(std::vector<Pose> points):points_(std::move(points)) {
  if(points_.size()>1 && distance(points_.front(),points_.back())<1e-6)points_.pop_back();
  if(points_.size()<3)throw std::invalid_argument("raceline requires at least three distinct points");
  for(std::size_t i=0;i<points_.size();++i){const auto&p=points_[i];
    if(!std::isfinite(p.x+p.y+p.yaw+p.v) || p.v<0 || distance(p,points_[(i+1)%points_.size()])<1e-6)
      throw std::invalid_argument("invalid/duplicate raceline waypoint");}
  rebuild();
}
Raceline Raceline::loadCsv(const std::string &path) {
  std::ifstream file(path);if(!file)throw std::runtime_error("cannot open raceline CSV: "+path);
  std::string line;std::vector<std::string> header;std::vector<Pose> points;
  auto split=[](const std::string&s){std::vector<std::string> fields;std::stringstream input(s);std::string f;
    while(std::getline(input,f,',')){if(!f.empty() && f.back()=='\r')f.pop_back();fields.push_back(f);}return fields;};
  if(!std::getline(file,line))throw std::runtime_error("empty raceline CSV");
  header=split(line);
  auto column=[&](const std::string&name){auto it=std::find(header.begin(),header.end(),name);
    if(it==header.end())throw std::runtime_error("raceline CSV missing column "+name);
    return static_cast<std::size_t>(it-header.begin());};
  const auto x=column("x_m"),y=column("y_m"),yaw=column("psi_rad"),v=column("vx_mps");
  while(std::getline(file,line)){if(line.empty())continue;const auto row=split(line);
    if(row.size()!=header.size())throw std::runtime_error("invalid raceline CSV row");
    auto number=[&](std::size_t col){std::size_t consumed=0;double value=std::stod(row[col],&consumed);
      if(consumed!=row[col].size())throw std::runtime_error("invalid raceline CSV number");
      return value;};
    points.push_back({number(x),number(y),number(yaw),number(v)});
  }
  return Raceline(std::move(points));
}
void Raceline::rebuild() {
  arc_.assign(points_.size()+1,0);
  for(std::size_t i=0;i<points_.size();++i)arc_[i+1]=arc_[i]+distance(points_[i],points_[(i+1)%points_.size()]);
  length_=arc_.back();
}
Pose Raceline::sample(double s)const {
  s=std::fmod(s,length_);if(s<0)s+=length_;
  const auto i=std::min(points_.size()-1,static_cast<std::size_t>(std::upper_bound(arc_.begin(),arc_.end(),s)-arc_.begin()-1));
  const auto&a=points_[i];const auto&b=points_[(i+1)%points_.size()];const double f=(s-arc_[i])/(arc_[i+1]-arc_[i]);
  return {a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),a.yaw+f*angle(b.yaw-a.yaw),a.v+f*(b.v-a.v)};
}
bool Raceline::project(const Pose &ego,double max_error,double max_heading_error,double forward,double backward,bool global) {
  double best=std::numeric_limits<double>::infinity(),best_s=progress_;
  for(std::size_t i=0;i<points_.size();++i){const auto&a=points_[i];const auto&b=points_[(i+1)%points_.size()];
    const double dx=b.x-a.x,dy=b.y-a.y,len=arc_[i+1]-arc_[i];
    const double f=std::clamp(((ego.x-a.x)*dx+(ego.y-a.y)*dy)/(len*len),0.0,1.0);
    double s=arc_[i]+f*len;
    if(!global){s+=std::round((progress_-s)/length_)*length_;if(s<progress_-backward || s>progress_+forward)continue;}
    if(std::abs(angle(sample(s).yaw-ego.yaw))>max_heading_error)continue;
    const double error=std::hypot(ego.x-a.x-f*dx,ego.y-a.y-f*dy);
    if(error<best){best=error;best_s=s;}
  }
  if(best>max_error)return false;
  progress_=best_s;return true;
}
bool Raceline::reset(const Pose &ego,double max_error,double max_heading_error) {
  initialized_=false;
  // Choose driving direction once from initial pose. Never reverse on a tracking miss.
  if(!project(ego,max_error,max_heading_error,0,0,true)) {
    std::reverse(points_.begin(),points_.end());for(auto&p:points_)p.yaw=angle(p.yaw+std::acos(-1.0));rebuild();
    if(!project(ego,max_error,max_heading_error,0,0,true))return false;
  }
  initialized_=true;return true;
}
std::vector<Pose> Raceline::reference(const Pose &ego,double lookahead,double step,double max_error,
    double max_heading_error,double forward,double backward) {
  if(!initialized_ || step<=0 || lookahead<=0 || forward>=length_/2 || backward>=length_/2 ||
     !project(ego,max_error,max_heading_error,forward,backward,false))return {};
  std::vector<Pose> out{ego};out.front().v=sample(progress_).v;
  for(double d=step;d<=lookahead;d+=step)out.push_back(sample(progress_+d));
  return out;
}
} // namespace obca
