#include "obca_navigation/ros_helpers.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <deque>
#include <optional>

namespace obca {
class Planner : public rclcpp::Node {
 public:
  Planner():Node("obca_planner"),c_(parameters(*this)),map_(c_),buffer_(get_clock()),listener_(buffer_) {
    frame_=declare_parameter<std::string>("map_frame","map");base_=declare_parameter<std::string>("base_frame","base_link");
    timeout_=declare_parameter("input_timeout",0.3);min_inlier_=declare_parameter("min_inlier_ratio",0.4);
    max_residual_=declare_parameter("max_icp_residual",0.35);period_=declare_parameter("planning_period",0.1);
    scan_convention_=declare_parameter<std::string>("scan_stamp_convention","begin");
    no_return_free_=declare_parameter("no_return_is_free",false);
    history_seconds_=declare_parameter("pose_history_seconds",2.0);max_queue_=declare_parameter("scan_queue_size",10);
    reset_position_tolerance_=declare_parameter("reset_position_tolerance",0.5);
    reset_yaw_tolerance_=declare_parameter("reset_yaw_tolerance",0.35);
    startup_straight_distance_=declare_parameter("startup_straight_distance",0.3);
    if(!std::isfinite(timeout_+period_+history_seconds_+min_inlier_+max_residual_+reset_position_tolerance_+reset_yaw_tolerance_) ||
       timeout_<=0 || period_<=0 || history_seconds_<=timeout_ || max_queue_<1 ||
       min_inlier_<0 || min_inlier_>1 || max_residual_<=0 || reset_position_tolerance_<=0 || reset_yaw_tolerance_<=0 ||
       !std::isfinite(startup_straight_distance_) || startup_straight_distance_<0 ||
       (scan_convention_!="begin" && scan_convention_!="end"))throw std::invalid_argument("invalid planner timing/quality settings");
    path_pub_=create_publisher<f110_msgs::msg::WpntArray>(declare_parameter<std::string>("waypoints_topic","/obca/waypoints"),1);
    visual_pub_=create_publisher<nav_msgs::msg::Path>(declare_parameter<std::string>("path_topic","/obca/path"),1);
    grid_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>(declare_parameter<std::string>("grid_topic","/obca/local_grid"),1);
    status_pub_=create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("status_topic","/obca/status"),1);
    initial_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(declare_parameter<std::string>("initial_pose_topic","/initialpose"),10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
        Pose p;if(m->header.frame_id!=frame_ || !poseFrom(m->pose.pose,p)){armed_=false;stop("invalid initial pose");return;}
        reset_=now().seconds();initial_=p;startup_=startup_straight_distance_>0;awaiting_pose_=true;armed_=true;healthy_=false;quality_reason_="waiting for ICP diagnostics after initial pose";diag_received_={};history_.clear();scans_.clear();previous_.clear();map_.clear();map_stamp_=0;stop("initial pose: waiting for new ICP scans");});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter<std::string>("pose_topic","/pf/pose/odom"),rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        Pose p;const double t=rclcpp::Time(m->header.stamp).seconds();
        if(!armed_ || t<reset_ || m->header.frame_id!=frame_ || !poseFrom(m->pose.pose,p) || !std::isfinite(m->twist.twist.linear.x))return;
        if(awaiting_pose_ && (distance(p,initial_)>reset_position_tolerance_ || std::abs(angle(p.yaw-initial_.yaw))>reset_yaw_tolerance_))return;
        if(!history_.empty() && t<=history_.back().first)return;
        awaiting_pose_=false;p.v=m->twist.twist.linear.x;history_.push_back({t,p});pose_received_=Steady::now();
        while(history_.size()>2 && t-history_.front().first>history_seconds_)history_.pop_front();});
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter<std::string>("scan_topic","/scan"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){if(armed_){scans_.push_back(m);while(scans_.size()>static_cast<std::size_t>(max_queue_))scans_.pop_front();}});
    diag_sub_=create_subscription<diagnostic_msgs::msg::DiagnosticArray>(declare_parameter<std::string>("diagnostics_topic","/kinematic_localization/diagnostics"),10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m){
        diag_stamp_=rclcpp::Time(m->header.stamp).seconds();healthy_=quality(*m,min_inlier_,max_residual_,&quality_reason_);diag_received_=Steady::now();
        if(diag_stamp_<reset_){healthy_=false;quality_reason_="ICP diagnostics predate initial pose";}
        if(!healthy_){map_.clear();map_stamp_=0;scans_.clear();previous_.clear();}
      });
    drive_sub_=create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(declare_parameter<std::string>("drive_topic","/obca/drive"),1,
      [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m){if(std::isfinite(m->drive.steering_angle))steering_=m->drive.steering_angle;});
    timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{tick();});
    RCLCPP_INFO(get_logger(),"Mapless OBCA: waiting for /initialpose; no global raceline or frozen map");
  }
 private:
  std::optional<Pose> at(double t)const {
    if(history_.empty() || t<history_.front().first || t>history_.back().first)return {};
    for(std::size_t i=1;i<history_.size();++i)if(history_[i].first>=t) {
      const auto&a=history_[i-1];const auto&b=history_[i];const double f=(t-a.first)/(b.first-a.first);
      return Pose{a.second.x+f*(b.second.x-a.second.x),a.second.y+f*(b.second.y-a.second.y),
        a.second.yaw+f*angle(b.second.yaw-a.second.yaw),a.second.v+f*(b.second.v-a.second.v)};
    }
    return history_.back().second;
  }
  bool integrate(const sensor_msgs::msg::LaserScan&m,double now_sec) {
    if(m.ranges.size()<2 || !std::isfinite(m.angle_min+m.angle_increment+m.time_increment+m.range_min+m.range_max) ||
       m.angle_increment==0 || m.time_increment<0 || m.range_min<0 || m.range_max<=m.range_min)return true;
    const double span=(m.ranges.size()-1)*m.time_increment;
    const double start=rclcpp::Time(m.header.stamp).seconds()-(scan_convention_=="end"?span:0),end=start+span;
    if(start<reset_ || end<=map_stamp_ || now_sec-end>timeout_)return true;
    if(history_.empty() || start<history_.front().first)return true;
    if(end>history_.back().first)return false;
    const auto a=at(start),b=at(end);if(!a || !b)return false;
    geometry_msgs::msg::TransformStamped extrinsic;
    try{extrinsic=buffer_.lookupTransform(base_,m.header.frame_id,tf2::TimePointZero);}catch(const tf2::TransformException&){return false;}
    geometry_msgs::msg::Pose ep;ep.position.x=extrinsic.transform.translation.x;ep.position.y=extrinsic.transform.translation.y;ep.orientation=extrinsic.transform.rotation;
    Pose sensor;if(!poseFrom(ep,sensor))return true;
    std::vector<Pose> endpoints;std::size_t usable=0;
    for(std::size_t i=0;i<m.ranges.size();++i) {
      const double measured=m.ranges[i];
      const bool hit=std::isfinite(measured) && measured>=m.range_min && measured<m.range_max;
      const bool clear=no_return_free_ && ((std::isinf(measured) && measured>0) || measured==m.range_max);
      if(!hit && !clear)continue;
      const double f=static_cast<double>(i)/(m.ranges.size()-1),yaw=a->yaw+f*angle(b->yaw-a->yaw);
      const double bx=a->x+f*(b->x-a->x),by=a->y+f*(b->y-a->y);
      const double ox=bx+std::cos(yaw)*sensor.x-std::sin(yaw)*sensor.y,oy=by+std::sin(yaw)*sensor.x+std::cos(yaw)*sensor.y;
      const double range=std::min(hit?measured:static_cast<double>(m.range_max),c_.map_radius);
      const double theta=yaw+sensor.yaw+m.angle_min+i*m.angle_increment;
      Pose p{ox+range*std::cos(theta),oy+range*std::sin(theta),0,0};
      map_.ray(ox,oy,p.x,p.y,end);++usable;
      if(hit && measured<=c_.map_radius)endpoints.push_back(p);
    }
    // Apply all endpoints after free rays, so a neighbouring beam cannot erase a hit in this scan.
    for(const auto&p:endpoints)map_.hit(p.x,p.y,end);
    if(usable){map_.ownFootprint(*b,end);map_stamp_=end;map_received_=Steady::now();}
    return true;
  }
  void status(const std::string&s){std_msgs::msg::String msg;msg.data=s;status_pub_->publish(msg);}
  void stop(const std::string&s){f110_msgs::msg::WpntArray m;m.header.frame_id=frame_;m.header.stamp=now();path_pub_->publish(m);
    nav_msgs::msg::Path p;p.header=m.header;visual_pub_->publish(p);previous_.clear();status(s);}
  void tick() {
    const double t=now().seconds();
    if(last_clock_>0 && t<last_clock_){armed_=false;map_.clear();history_.clear();scans_.clear();stop("clock reset: initial pose required");}
    last_clock_=t;
    if(t<=0){stop("ROS clock not started: check /clock or set use_sim_time:=false for standard gym");return;}
    if(!armed_ || awaiting_pose_ || history_.empty()){stop("waiting for initial pose and ICP");return;}
    if(diag_received_==Steady::time_point{}){stop("waiting for ICP diagnostics");return;}
    if(t<diag_stamp_ || t<history_.back().first){stop("localization stamp is in the future: check consistent use_sim_time and /clock");return;}
    if(age(diag_received_)>timeout_ || t-diag_stamp_>timeout_){
      stop("stale ICP diagnostics: receipt_age="+std::to_string(age(diag_received_))+"; stamp_age="+std::to_string(t-diag_stamp_));return;}
    if(age(pose_received_)>timeout_ || t-history_.back().first>timeout_){
      stop("stale ICP pose: receipt_age="+std::to_string(age(pose_received_))+"; stamp_age="+std::to_string(t-history_.back().first));return;}
    if(!healthy_){stop(quality_reason_);return;}
    while(!scans_.empty()){if(!integrate(*scans_.front(),t))break;scans_.pop_front();}
    if(map_stamp_<=0 || t-map_stamp_>timeout_ || age(map_received_)>timeout_){stop("waiting for fresh synchronized scan/TF");return;}
    const Pose ego=history_.back().second;
    Grid grid=map_.snapshot(ego,t);
    nav_msgs::msg::OccupancyGrid gm;gm.header.frame_id=frame_;gm.header.stamp=now();gm.info.resolution=grid.resolution;gm.info.width=grid.width;gm.info.height=grid.height;
    gm.info.origin.position.x=grid.x0;gm.info.origin.position.y=grid.y0;gm.info.origin.orientation.w=1;gm.data.assign(grid.cells.begin(),grid.cells.end());grid_pub_->publish(gm);
    if(!grid.footprint(ego,c_)){stop("vehicle footprint not in observed free space");return;}
    if(startup_ && (ego.x-initial_.x)*std::cos(initial_.yaw)+(ego.y-initial_.y)*std::sin(initial_.yaw)>=startup_straight_distance_) {
      startup_=false;previous_.clear();
    }
    const auto ref=startup_?straightReference(grid,ego,c_):reference(grid,ego,c_,previous_);
    if(ref.size()<2){stop("reference missing: no connected route in observed free space");return;}
    const double reach=c_.max_speed*c_.horizon*c_.dt+std::hypot(std::max(c_.front,c_.rear),c_.half_width)+
      std::sqrt(2.0)*(c_.margin+c_.validation_step)+c_.validation_tolerance;
    const auto boxes=obstacles(grid,ego,reach);
    if(boxes.size()>static_cast<std::size_t>(c_.max_obstacles)){stop("obstacle budget exceeded: count="+
      std::to_string(boxes.size())+"; limit="+std::to_string(c_.max_obstacles)+"; no obstacle was discarded");return;}
    const double input_stamp=history_.back().first;
    auto solution=solve(ego,steering_,ref,boxes,c_,previous_,startup_);
    std::string reason;
    if(!solution.success){stop(solution.reason+"; solve_ms="+std::to_string(solution.elapsed_ms)+"; pairs="+std::to_string(solution.collision_pairs));return;}
    if(now().seconds()-map_stamp_>timeout_ || now().seconds()-input_stamp>timeout_ || age(pose_received_)>timeout_ || age(diag_received_)>timeout_){stop("inputs expired during optimization");return;}
    if(!validatePath(grid,solution.states,c_,reason)){stop(reason);return;}
    f110_msgs::msg::WpntArray msg;msg.header.frame_id=frame_;
    // Timestamp the input state, not solve completion; consumers account for optimization age.
    msg.header.stamp=rclcpp::Time(static_cast<int64_t>(input_stamp*1e9));
    nav_msgs::msg::Path path;path.header=msg.header;double s=0;
    for(std::size_t i=0;i<solution.states.size();++i){const auto&p=solution.states[i];if(i)s+=distance(solution.states[i-1],p);
      f110_msgs::msg::Wpnt w;w.id=static_cast<int>(i);w.s_m=s;w.x_m=p.x;w.y_m=p.y;w.psi_rad=p.yaw;w.vx_mps=p.v;w.ax_mps2=p.acceleration;w.kappa_radpm=std::tan(p.steering)/c_.wheelbase;msg.wpnts.push_back(w);
      geometry_msgs::msg::PoseStamped ps;ps.header=msg.header;ps.pose=poseMessage(p);path.poses.push_back(ps);}
    path_pub_->publish(msg);visual_pub_->publish(path);previous_=solution.states;
    status(std::string("valid; mode=")+(startup_?"startup_straight":"obca")+"; solve_ms="+std::to_string(solution.elapsed_ms)+"; boxes="+std::to_string(boxes.size())+"; pairs="+std::to_string(solution.collision_pairs));
  }
  Config c_;LocalMap map_;tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  std::string frame_,base_,scan_convention_,quality_reason_;double timeout_{},min_inlier_{},max_residual_{},period_{},history_seconds_{},reset_position_tolerance_{},reset_yaw_tolerance_{};
  int max_queue_{};bool no_return_free_{},armed_{false},awaiting_pose_{true},healthy_{false};
  double startup_straight_distance_{};bool startup_{false};
  double reset_{},map_stamp_{},diag_stamp_{},last_clock_{},steering_{};Pose initial_;
  Steady::time_point pose_received_{},diag_received_{},map_received_{};
  std::deque<std::pair<double,Pose>> history_;std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr> scans_;std::vector<State>previous_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr path_pub_;rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visual_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace obca
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<obca::Planner>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("obca_planner"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
