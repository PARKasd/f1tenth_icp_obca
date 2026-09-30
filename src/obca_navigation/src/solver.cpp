#include "obca_navigation/core.hpp"
#include <IpStdCInterface.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <memory>

namespace obca {
namespace {
using Clock=std::chrono::steady_clock;
struct Problem {
  Config c; Pose ego; double last_steering;
  std::vector<Box> boxes;
  std::vector<Pose> targets;
  std::vector<double> lo,hi,gl,gu;
  Clock::time_point start=Clock::now();
  int controls()const{return 4*(c.horizon+1);}
  int dual(int i,int j)const{return controls()+2*c.horizon+8*(i*static_cast<int>(boxes.size())+j);}
  int variables()const{return controls()+2*c.horizon+8*(c.horizon+1)*static_cast<int>(boxes.size());}
  int constraints()const{return 6*c.horizon+4*(c.horizon+1)*static_cast<int>(boxes.size());}

  // Both sparsity and values use this exact traversal. No finite differencing at runtime.
  template<class Add> void jacobian(const double *z,Add add)const {
    const double dummy[4]={0,0,0,0};
    for(int i=0;i<c.horizon;++i) {
      const int k=4*i,u=controls()+2*i,r=6*i;
      const double *p=z?z+k:dummy;
      const double d=z?z[u]:0,t=std::tan(d),sec=1+t*t;
      add(r,k,-1);add(r,k+4,1);add(r,k+2,c.dt*p[3]*std::sin(p[2]));add(r,k+3,-c.dt*std::cos(p[2]));
      add(r+1,k+1,-1);add(r+1,k+5,1);add(r+1,k+2,-c.dt*p[3]*std::cos(p[2]));add(r+1,k+3,-c.dt*std::sin(p[2]));
      add(r+2,k+2,-1);add(r+2,k+6,1);add(r+2,k+3,-c.dt*t/c.wheelbase);add(r+2,u,-c.dt*p[3]*sec/c.wheelbase);
      add(r+3,k+3,-1);add(r+3,k+7,1);add(r+3,u+1,-c.dt);
      add(r+4,u,1);if(i) add(r+4,u-2,-1);
      add(r+5,k+3,2*p[3]*t/c.wheelbase);add(r+5,u,p[3]*p[3]*sec/c.wheelbase);
    }
    for(int i=0;i<=c.horizon;++i) for(int j=0;j<static_cast<int>(boxes.size());++j) {
      const int k=4*i,d=dual(i,j),r=6*c.horizon+4*(i*static_cast<int>(boxes.size())+j);
      const double theta=z?z[k+2]:0,cs=std::cos(theta),sn=std::sin(theta);
      const double nx=z?z[d]-z[d+2]:0,ny=z?z[d+1]-z[d+3]:0;
      add(r,d,2*nx);add(r,d+2,-2*nx);add(r,d+1,2*ny);add(r,d+3,-2*ny);
      add(r+1,d+4,1);add(r+1,d+6,-1);add(r+1,d,cs);add(r+1,d+2,-cs);add(r+1,d+1,sn);add(r+1,d+3,-sn);
      add(r+1,k+2,-sn*nx+cs*ny);
      add(r+2,d+5,1);add(r+2,d+7,-1);add(r+2,d,-sn);add(r+2,d+2,sn);add(r+2,d+1,cs);add(r+2,d+3,-cs);
      add(r+2,k+2,-cs*nx-sn*ny);
      const Box &b=boxes[j];const double x=z?z[k]:0,y=z?z[k+1]:0;
      add(r+3,k,nx);add(r+3,k+1,ny);
      add(r+3,d,x-b.xmax);add(r+3,d+1,y-b.ymax);add(r+3,d+2,b.xmin-x);add(r+3,d+3,b.ymin-y);
      add(r+3,d+4,-c.front);add(r+3,d+5,-c.half_width);add(r+3,d+6,-c.rear);add(r+3,d+7,-c.half_width);
    }
  }
  void values(const double *z,double *g)const {
    for(int i=0;i<c.horizon;++i) {
      const int k=4*i,u=controls()+2*i,r=6*i;
      g[r]=z[k+4]-z[k]-c.dt*z[k+3]*std::cos(z[k+2]);
      g[r+1]=z[k+5]-z[k+1]-c.dt*z[k+3]*std::sin(z[k+2]);
      g[r+2]=z[k+6]-z[k+2]-c.dt*z[k+3]*std::tan(z[u])/c.wheelbase;
      g[r+3]=z[k+7]-z[k+3]-c.dt*z[u+1];
      g[r+4]=z[u]-(i?z[u-2]:last_steering);
      g[r+5]=z[k+3]*z[k+3]*std::tan(z[u])/c.wheelbase;
    }
    for(int i=0;i<=c.horizon;++i) for(int j=0;j<static_cast<int>(boxes.size());++j) {
      const int k=4*i,d=dual(i,j),r=6*c.horizon+4*(i*static_cast<int>(boxes.size())+j);
      const double nx=z[d]-z[d+2],ny=z[d+1]-z[d+3],cs=std::cos(z[k+2]),sn=std::sin(z[k+2]);
      const Box &b=boxes[j];
      g[r]=nx*nx+ny*ny;
      g[r+1]=z[d+4]-z[d+6]+cs*nx+sn*ny;
      g[r+2]=z[d+5]-z[d+7]-sn*nx+cs*ny;
      g[r+3]=nx*z[k]+ny*z[k+1]-b.xmax*z[d]-b.ymax*z[d+1]+b.xmin*z[d+2]+b.ymin*z[d+3]
        -c.front*z[d+4]-c.half_width*z[d+5]-c.rear*z[d+6]-c.half_width*z[d+7];
    }
  }
  double objective(const double *z,double *grad)const {
    if(grad)std::fill(grad,grad+variables(),0.0);
    double value=0;
    auto sq=[&](int index,double error,double weight){value+=weight*error*error;if(grad)grad[index]+=2*weight*error;};
    for(int i=0;i<=c.horizon;++i) {
      const int k=4*i;
      sq(k,z[k]-targets[i].x,c.position_weight);sq(k+1,z[k+1]-targets[i].y,c.position_weight);
      // Smooth periodic heading cost avoids a discontinuity at +-pi.
      const double e=z[k+2]-targets[i].yaw;
      value+=c.heading_weight*(1-std::cos(e));if(grad)grad[k+2]+=c.heading_weight*std::sin(e);
      sq(k+3,z[k+3]-targets[i].v,c.speed_weight);
    }
    for(int i=0;i<c.horizon;++i) {
      const int u=controls()+2*i;
      sq(u,z[u],c.steering_weight);sq(u+1,z[u+1],c.acceleration_weight);
      const double e=z[u]-(i?z[u-2]:last_steering);
      sq(u,e,c.smooth_weight);if(grad && i)grad[u-2]-=2*c.smooth_weight*e;
    }
    return value;
  }
};
Bool eval_f(Index,Number*x,Bool,Number*f,UserDataPtr data){*f=static_cast<Problem*>(data)->objective(x,nullptr);return std::isfinite(*f);}
Bool eval_grad(Index,Number*x,Bool,Number*g,UserDataPtr data){static_cast<Problem*>(data)->objective(x,g);return true;}
Bool eval_g(Index,Number*x,Bool,Index,Number*g,UserDataPtr data){static_cast<Problem*>(data)->values(x,g);return true;}
Bool eval_jac(Index,Number*x,Bool,Index,Index,Index*rows,Index*cols,Number*values,UserDataPtr data){
  int i=0;static_cast<Problem*>(data)->jacobian(values?x:nullptr,[&](int r,int c,double v){
    if(values)values[i]=v;else{rows[i]=r;cols[i]=c;}++i;});return true;
}
Bool eval_h(Index,Number*,Bool,Number,Index,Number*,Bool,Index,Index*,Index*,Number*,UserDataPtr){return true;}
Bool intermediate(Index,Index,Number,Number,Number,Number,Number,Number,Number,Number,Index,UserDataPtr data){
  auto &p=*static_cast<Problem*>(data);
  return std::chrono::duration<double>(Clock::now()-p.start).count()<p.c.solve_seconds;
}
std::vector<Pose> sampleTargets(const std::vector<Pose>&ref,const Pose&ego,const Config&c) {
  std::vector<double> s(ref.size(),0);
  for(std::size_t i=1;i<ref.size();++i)s[i]=s[i-1]+distance(ref[i-1],ref[i]);
  std::vector<Pose> out;double progress=0,speed=std::max(0.0,ego.v);
  for(int i=0;i<=c.horizon;++i) {
    auto upper=std::upper_bound(s.begin(),s.end(),progress);
    const std::size_t j=std::min<std::size_t>(std::max<std::size_t>(1,upper-s.begin()),ref.size()-1);
    const double fraction=std::clamp((progress-s[j-1])/std::max(1e-9,s[j]-s[j-1]),0.0,1.0);
    Pose p{ref[j-1].x+fraction*(ref[j].x-ref[j-1].x),ref[j-1].y+fraction*(ref[j].y-ref[j-1].y),ref[j-1].yaw,speed};
    p.yaw=ego.yaw+angle(p.yaw-ego.yaw);out.push_back(p);
    progress=std::min(s.back(),progress+speed*c.dt);
    speed=std::min({c.max_speed,speed+c.max_accel*c.dt,std::max(0.0,(c.horizon-i-1)*c.dt*c.max_decel),std::sqrt(2*c.max_decel*std::max(0.0,s.back()-progress))});
  }
  out.back().v=0;return out;
}
} // namespace
#ifdef OBCA_TESTING
double derivativeError() {
  Config c;c.horizon=4;
  Problem p{c,{},0.1,{{1,1,2,2}},std::vector<Pose>(5),{},{},{},{},Clock::now()};
  std::vector<double> z(p.variables(),0.2),ga(p.constraints()),gb(p.constraints());
  for(int i=0;i<p.variables();++i)z[i]+=0.01*(i%7);
  std::vector<std::vector<double>> jac(p.constraints(),std::vector<double>(p.variables(),0));
  p.jacobian(z.data(),[&](int r,int k,double v){jac[r][k]+=v;});
  std::vector<double> gradient(p.variables());p.objective(z.data(),gradient.data());
  double error=0;constexpr double step=1e-6;
  for(int k=0;k<p.variables();++k) {
    z[k]+=step;p.values(z.data(),ga.data());const double fa=p.objective(z.data(),nullptr);
    z[k]-=2*step;p.values(z.data(),gb.data());const double fb=p.objective(z.data(),nullptr);z[k]+=step;
    error=std::max(error,std::abs(gradient[k]-(fa-fb)/(2*step)));
    for(int r=0;r<p.constraints();++r)error=std::max(error,std::abs(jac[r][k]-(ga[r]-gb[r])/(2*step)));
  }
  return error;
}
#endif
Solution solve(const Pose &ego,double steering,const std::vector<Pose> &ref,const std::vector<Box> &boxes,const Config &c,const std::vector<State> &warm) {
  Solution result; const auto started=Clock::now();
  c.validate();
  if(ref.size()<2 || boxes.size()>static_cast<std::size_t>(c.max_obstacles)) {result.reason="reference missing or obstacle budget exceeded";return result;}
  if(!std::isfinite(ego.x+ego.y+ego.yaw+ego.v+steering) || ego.v<0 || ego.v>c.max_speed+c.validation_tolerance || std::abs(steering)>c.max_steering) {result.reason="initial state outside configured limits";return result;}
  Problem p{c,ego,steering,boxes,sampleTargets(ref,ego,c),{},{},{},{},started};
  const int n=p.variables(),m=p.constraints();
  const double inf=1e19,reach=c.max_speed*c.horizon*c.dt;
  p.lo.assign(n,0);p.hi.assign(n,inf);p.gl.assign(m,0);p.gu.assign(m,0);
  std::vector<double> z(n,0),g(m,0);
  std::size_t warm_start=0;
  for(std::size_t i=1;i<warm.size();++i)if(distance(warm[i],ego)<distance(warm[warm_start],ego))warm_start=i;
  for(int i=0;i<=c.horizon;++i) {
    const int k=4*i;
    Pose seed=p.targets[i];
    if(!warm.empty()){seed=warm[std::min(warm_start+i,warm.size()-1)];seed.yaw=ego.yaw+angle(seed.yaw-ego.yaw);}
    const double values[]={seed.x,seed.y,seed.yaw,seed.v};
    for(int a=0;a<4;++a)z[k+a]=values[a];
    p.lo[k]=ego.x-reach;p.hi[k]=ego.x+reach;p.lo[k+1]=ego.y-reach;p.hi[k+1]=ego.y+reach;
    p.lo[k+2]=ego.yaw-2*std::acos(-1.0);p.hi[k+2]=ego.yaw+2*std::acos(-1.0);p.hi[k+3]=c.max_speed;
    for(int j=0;j<static_cast<int>(boxes.size());++j) {
      const int d=p.dual(i,j),r=6*c.horizon+4*(i*static_cast<int>(boxes.size())+j);
      // The independent swept validator expands both body axes. A Euclidean
      // clearance of sqrt(2)*padding also covers the expanded corners.
      p.gl[r]=-inf;p.gu[r]=1;
      p.gl[r+3]=std::sqrt(2.0)*(c.margin+c.validation_step)+c.validation_tolerance;
      p.gu[r+3]=inf;
      const auto &b=boxes[j];double nx=seed.x-std::clamp(seed.x,b.xmin,b.xmax),ny=seed.y-std::clamp(seed.y,b.ymin,b.ymax);
      const double norm=std::hypot(nx,ny);if(norm>1e-9){nx/=norm;ny/=norm;}else{nx=1;ny=0;}
      z[d]=std::max(0.0,nx);z[d+1]=std::max(0.0,ny);z[d+2]=std::max(0.0,-nx);z[d+3]=std::max(0.0,-ny);
      const double mx=-std::cos(seed.yaw)*nx-std::sin(seed.yaw)*ny,my=std::sin(seed.yaw)*nx-std::cos(seed.yaw)*ny;
      z[d+4]=std::max(0.0,mx);z[d+5]=std::max(0.0,my);z[d+6]=std::max(0.0,-mx);z[d+7]=std::max(0.0,-my);
    }
  }
  const double initial[]={ego.x,ego.y,ego.yaw,ego.v};
  for(int k=0;k<4;++k)p.lo[k]=p.hi[k]=z[k]=initial[k];
  p.lo[4*c.horizon+3]=p.hi[4*c.horizon+3]=z[4*c.horizon+3]=0;
  for(int i=0;i<c.horizon;++i) {
    const int u=p.controls()+2*i,r=6*i;
    p.lo[u]=-c.max_steering;p.hi[u]=c.max_steering;p.lo[u+1]=-c.max_decel;p.hi[u+1]=c.max_accel;
    z[u]=steering;z[u+1]=std::clamp((z[4*(i+1)+3]-z[4*i+3])/c.dt,-c.max_decel,c.max_accel);
    p.gl[r+4]=-c.max_steering_rate*c.dt;p.gu[r+4]=c.max_steering_rate*c.dt;
    p.gl[r+5]=-c.max_lateral_accel;p.gu[r+5]=c.max_lateral_accel;
  }
  int nnz=0;p.jacobian(nullptr,[&](int,int,double){++nnz;});
  IpoptProblem raw=CreateIpoptProblem(n,p.lo.data(),p.hi.data(),m,p.gl.data(),p.gu.data(),nnz,0,0,eval_f,eval_g,eval_grad,eval_jac,eval_h);
  if(!raw){result.reason="Ipopt creation failed";return result;}
  struct Guard{IpoptProblem p;~Guard(){FreeIpoptProblem(p);}} guard{raw};
  AddIpoptStrOption(raw,const_cast<char*>("hessian_approximation"),const_cast<char*>("limited-memory"));
  AddIpoptStrOption(raw,const_cast<char*>("mu_strategy"),const_cast<char*>("adaptive"));
  AddIpoptStrOption(raw,const_cast<char*>("sb"),const_cast<char*>("yes"));
  AddIpoptIntOption(raw,const_cast<char*>("print_level"),0);
  AddIpoptIntOption(raw,const_cast<char*>("max_iter"),c.max_iterations);
  AddIpoptNumOption(raw,const_cast<char*>("tol"),c.tolerance);
  AddIpoptNumOption(raw,const_cast<char*>("max_cpu_time"),c.solve_seconds);
  SetIntermediateCallback(raw,intermediate);
  double objective=0;
  const auto status=IpoptSolve(raw,z.data(),g.data(),&objective,nullptr,nullptr,nullptr,&p);
  result.elapsed_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
  p.values(z.data(),g.data());
  for(int i=0;i<n;++i) {
    if(!std::isfinite(z[i]))result.max_violation=inf;
    result.max_violation=std::max({result.max_violation,p.lo[i]-z[i],z[i]-p.hi[i]});
  }
  for(int i=0;i<m;++i) {
    if(!std::isfinite(g[i]))result.max_violation=inf;
    result.max_violation=std::max({result.max_violation,p.gl[i]-g[i],g[i]-p.gu[i]});
  }
  result.success=(status==Solve_Succeeded || status==Solved_To_Acceptable_Level) &&
    result.max_violation<=c.validation_tolerance && result.elapsed_ms<=c.solve_seconds*1000;
  result.reason=result.success?"solved":"Ipopt status="+std::to_string(static_cast<int>(status))+", residual="+std::to_string(result.max_violation);
  if(result.success)for(int i=0;i<=c.horizon;++i) {
    State state;state.x=z[4*i];state.y=z[4*i+1];state.yaw=z[4*i+2];state.v=std::clamp(z[4*i+3],0.0,c.max_speed);
    if(i<c.horizon){state.steering=std::clamp(z[p.controls()+2*i],-c.max_steering,c.max_steering);state.acceleration=std::clamp(z[p.controls()+2*i+1],-c.max_decel,c.max_accel);}
    result.states.push_back(state);
  }
  return result;
}
} // namespace obca
