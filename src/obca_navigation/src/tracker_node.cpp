#include "obca_navigation/ros_helpers.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>

namespace obca {
class Tracker:public rclcpp::Node {
 public:
  Tracker():Node("obca_tracker"),c_(parameters(*this)) {
    frame_=declare_parameter<std::string>("map_frame","map");base_=declare_parameter<std::string>("base_frame","base_link");
    timeout_=declare_parameter("input_timeout",0.3);path_timeout_=declare_parameter("path_timeout",0.3);
    period_=declare_parameter("control_period",0.02);lookahead_=declare_parameter("lookahead",0.45);
    max_error_=declare_parameter("max_tracking_error",0.25);goal_tolerance_=declare_parameter("goal_tolerance",0.12);
    min_inlier_=declare_parameter("min_inlier_ratio",0.4);max_residual_=declare_parameter("max_icp_residual",0.35);
    if(!std::isfinite(timeout_+path_timeout_+period_+lookahead_+max_error_+goal_tolerance_+min_inlier_+max_residual_) ||
       timeout_<=0 || path_timeout_<=0 || period_<=0 || lookahead_<=0 || max_error_<=0 || goal_tolerance_<=0 ||
       min_inlier_<0 || min_inlier_>1 || max_residual_<=0)throw std::invalid_argument("invalid tracker configuration");
    pub_=create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(declare_parameter<std::string>("drive_topic","/obca/drive"),1);
    initial_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(declare_parameter<std::string>("initial_pose_topic","/initialpose"),10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){Pose p;armed_=m->header.frame_id==frame_ && poseFrom(m->pose.pose,p);reset_=now().seconds();path_.clear();have_pose_=false;healthy_=false;commandStop();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter<std::string>("pose_topic","/pf/pose/odom"),10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){Pose p;double stamp=rclcpp::Time(m->header.stamp).seconds();
        if(!armed_ || stamp<reset_ || m->header.frame_id!=frame_ || !poseFrom(m->pose.pose,p) || !std::isfinite(m->twist.twist.linear.x))return;
        if(have_pose_ && stamp<=pose_stamp_)return;
        ego_=p;ego_.v=m->twist.twist.linear.x;pose_stamp_=stamp;pose_received_=Steady::now();have_pose_=true;});
    path_sub_=create_subscription<f110_msgs::msg::WpntArray>(declare_parameter<std::string>("waypoints_topic","/obca/waypoints"),1,
      [this](f110_msgs::msg::WpntArray::ConstSharedPtr m){
        path_.clear();const double stamp=rclcpp::Time(m->header.stamp).seconds();
        if(!armed_ || m->header.frame_id!=frame_ || stamp<reset_ || m->wpnts.size()<2)return;
        for(const auto&w:m->wpnts){if(!std::isfinite(w.x_m+w.y_m+w.psi_rad+w.vx_mps+w.kappa_radpm+w.s_m) || w.vx_mps<0 || w.vx_mps>c_.max_speed+c_.validation_tolerance){path_.clear();return;}
          if(!path_.empty() && w.s_m<path_.back().s_m){path_.clear();return;}path_.push_back(w);}
        if(path_.back().vx_mps>c_.validation_tolerance){path_.clear();return;}
        path_stamp_=stamp;path_received_=Steady::now();});
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter<std::string>("scan_topic","/scan"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
        if(std::any_of(m->ranges.begin(),m->ranges.end(),[&](float r){return std::isfinite(r)&&r>=m->range_min&&r<m->range_max;})){
          scan_stamp_=rclcpp::Time(m->header.stamp).seconds();scan_received_=Steady::now();}});
    diag_sub_=create_subscription<diagnostic_msgs::msg::DiagnosticArray>(declare_parameter<std::string>("diagnostics_topic","/kinematic_localization/diagnostics"),10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m){diag_stamp_=rclcpp::Time(m->header.stamp).seconds();healthy_=diag_stamp_>=reset_ && quality(*m,min_inlier_,max_residual_);diag_received_=Steady::now();});
    last_tick_=Steady::now();timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{tick();});
  }
 private:
  void publish(double speed,double steer,double acceleration){ackermann_msgs::msg::AckermannDriveStamped m;m.header.stamp=now();m.header.frame_id=base_;
    m.drive.speed=speed;m.drive.steering_angle=steer;m.drive.steering_angle_velocity=c_.max_steering_rate;m.drive.acceleration=acceleration;pub_->publish(m);speed_=speed;steering_=steer;}
  void commandStop(){publish(0,steering_,c_.max_decel);}
  bool fresh(double stamp,Steady::time_point received,double limit,double t)const{return stamp>=reset_ && t>=stamp && t-stamp<=limit && age(received)<=limit;}
  void tick(){
    const auto tick=Steady::now();const double elapsed=std::chrono::duration<double>(tick-last_tick_).count();last_tick_=tick;
    const double t=now().seconds();if(last_clock_>0 && t<last_clock_){armed_=false;path_.clear();}last_clock_=t;
    if(!armed_ || !have_pose_ || !healthy_ || path_.empty() || elapsed>timeout_ ||
       !fresh(pose_stamp_,pose_received_,timeout_,t) || !fresh(path_stamp_,path_received_,path_timeout_,t) ||
       !fresh(scan_stamp_,scan_received_,timeout_,t) || !fresh(diag_stamp_,diag_received_,timeout_,t)){commandStop();return;}
    std::size_t closest=0;double error=std::numeric_limits<double>::infinity(),progress=0;
    for(std::size_t i=0;i+1<path_.size();++i){const auto&a=path_[i];const auto&b=path_[i+1];const double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m;
      const double f=std::clamp(((ego_.x-a.x_m)*dx+(ego_.y-a.y_m)*dy)/std::max(1e-12,dx*dx+dy*dy),0.0,1.0);
      const double e=std::hypot(ego_.x-a.x_m-f*dx,ego_.y-a.y_m-f*dy);
      if(e<error){error=e;closest=i;progress=a.s_m+f*(b.s_m-a.s_m);}}
    const double remaining=path_.back().s_m-progress;
    if(error>max_error_ || remaining<=goal_tolerance_ || std::abs(angle(path_[closest].psi_rad-ego_.yaw))>std::acos(-1.0)/2){commandStop();return;}
    std::size_t target=closest+1;while(target+1<path_.size() && path_[target].s_m-progress<lookahead_)++target;
    const double dx=path_[target].x_m-ego_.x,dy=path_[target].y_m-ego_.y;
    const double local_x=std::cos(ego_.yaw)*dx+std::sin(ego_.yaw)*dy,local_y=-std::sin(ego_.yaw)*dx+std::cos(ego_.yaw)*dy;
    if(local_x<=0){commandStop();return;}
    const double desired=std::clamp(std::atan2(2*c_.wheelbase*local_y,dx*dx+dy*dy),-c_.max_steering,c_.max_steering);
    const double steer=std::clamp(desired,steering_-c_.max_steering_rate*elapsed,steering_+c_.max_steering_rate*elapsed);
    double speed=std::min({c_.max_speed,path_[closest+1].vx_mps,std::sqrt(2*c_.max_decel*std::max(0.0,remaining-goal_tolerance_))});
    speed=std::min(speed,std::sqrt(c_.max_lateral_accel*c_.wheelbase/std::max(1e-9,std::abs(std::tan(steer)))));
    speed=std::min(speed,speed_+c_.max_accel*elapsed);
    publish(speed,steer,speed<speed_?c_.max_decel:c_.max_accel);
  }
  Config c_;std::string frame_,base_;double timeout_{},path_timeout_{},period_{},lookahead_{},max_error_{},goal_tolerance_{},min_inlier_{},max_residual_{};
  bool armed_{false},have_pose_{false},healthy_{false};Pose ego_;std::vector<f110_msgs::msg::Wpnt>path_;
  double reset_{},pose_stamp_{},path_stamp_{},scan_stamp_{},diag_stamp_{},last_clock_{},speed_{},steering_{};
  Steady::time_point last_tick_{},pose_received_{},path_received_{},scan_received_{},diag_received_{};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace obca
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<obca::Tracker>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("obca_tracker"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
