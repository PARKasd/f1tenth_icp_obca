#pragma once
#include <cmath>
#include <map>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace obca {
struct Pose { double x{}, y{}, yaw{}, v{}; };
struct Box { double xmin{}, ymin{}, xmax{}, ymax{}; };
struct State : Pose { double steering{}, acceleration{}; };
struct Config {
  int horizon{16}, max_obstacles{64}, max_iterations{120};
  double dt{0.2}, wheelbase{0.3302}, front{0.38}, rear{0.14}, half_width{0.16};
  double margin{0.06}, max_speed{0.8}, max_steering{0.41}, max_steering_rate{1.0};
  double max_accel{1.0}, max_decel{1.5}, max_lateral_accel{1.5};
  double solve_seconds{0.15}, tolerance{1e-5}, validation_tolerance{1e-3};
  double position_weight{8.0}, heading_weight{0.5}, speed_weight{1.0};
  double steering_weight{0.2}, acceleration_weight{0.1}, smooth_weight{1.0};
  double grid_resolution{0.1}, map_radius{6.0}, map_ttl{2.0};
  double reference_distance{2.5}, reference_clearance{0.23}, validation_step{0.04};
  double seed_lookahead{0.45};
  double goal_forward_weight{0.5}, goal_lateral_weight{0.1}, goal_path_weight{0.1}, goal_continuity_weight{0.3};
  double recovery_stationary_speed{0.02};
  double reference_wall_weight{0.0}, goal_clearance_weight{0.0};
  double goal_progress_weight{1.0}, goal_route_clearance_weight{0.2};
  double goal_turn_weight{0.1}, goal_continuation_weight{0.5};
  double goal_clearance_target{1.0}, goal_continuation_distance{0.8};
  void validate() const;
};
// Conservative travel bound including acceleration, terminal braking and residual tolerance.
double reachableDistance(double speed, const Config &config);
// Brake before a future steering corner, rather than only limiting current steering.
double cornerPreviewSpeed(const std::vector<State> &path, double progress, const Config &config);
// A rolling local horizon is not a final destination. Reserve at most 10% of
// its length for stopping, so short validated observation paths remain usable.
double localStopBuffer(double path_length, double goal_tolerance);
double angle(double a);
double distance(const Pose &a, const Pose &b);
bool overlap(const Pose &p, double front, double rear, double half_width, const Box &b);
struct Grid {
  int width{}, height{};
  double resolution{}, x0{}, y0{};
  std::vector<int> cells; // -1 unknown, 0 observed free, 100 occupied
  Pose known_body{}; // The robot itself is known to occupy this region, even in the rear blind spot.
  bool has_known_body{false};
  int index(double x, double y) const;
  Pose center(int index) const;
  bool footprint(const Pose &p, const Config &c) const;
};
class LocalMap {
 public:
  explicit LocalMap(Config c) : c_(c) {}
  void clear() { cells_.clear(); }
  void ray(double ox, double oy, double ex, double ey, double time);
  void hit(double x, double y, double time);
  void ownFootprint(const Pose &pose, double time);
  Grid snapshot(const Pose &p, double time);
 private:
  struct Cell { int value; double time; };
  Config c_;
  std::map<std::pair<int,int>, Cell> cells_;
};
// Closed raceline, with unwrapped progress to avoid switching hairpin branches.
class Raceline {
 public:
  explicit Raceline(std::vector<Pose> points);
  static Raceline loadCsv(const std::string &path);
  bool reset(const Pose &ego,double max_error,double max_heading_error);
  std::vector<Pose> reference(const Pose &ego,double lookahead,double step,
    double max_error,double max_heading_error,double forward_window,double backward_window);
  const std::vector<Pose>& points()const{return points_;}
  double progress()const{return progress_;}
  double length()const{return length_;}
 private:
  void rebuild();
  Pose sample(double progress)const;
  bool project(const Pose &ego,double max_error,double max_heading_error,
    double forward_window,double backward_window,bool global);
  std::vector<Pose> points_;
  std::vector<double> arc_;
  double length_{},progress_{};
  bool initialized_{false};
};
// Features are measured only along connected, observed-space search paths.
struct GoalFeatures {
  bool observed_connected{false}, has_previous{false};
  double progress_m{}, lateral_m{}, path_cost_m{};
  double goal_clearance_m{}, route_clearance_m{};
  double initial_heading_error_rad{}, heading_change_rad{}, continuation_m{};
  double previous_goal_distance_m{};
};
struct GoalEvaluation {
  bool eligible{false};
  Pose goal{};
  GoalFeatures features{};
  double total{-std::numeric_limits<double>::infinity()};
  double progress{}, direction{}, lateral{}, path_cost{}, clearance{};
  double route_clearance{}, turn{}, continuation{}, continuity{};
};
GoalEvaluation scoreLocalGoal(const GoalFeatures &features, const Config &config);
std::vector<Pose> reference(const Grid &grid, const Pose &ego, const Config &c,
                            const std::vector<State> &previous = {},
                            GoalEvaluation *evaluation = nullptr);
std::vector<Pose> straightReference(const Grid &grid, const Pose &ego, const Config &c);
std::vector<Box> obstacles(const Grid &grid, const Pose &ego, double reach);
bool validatePath(const Grid &grid, const std::vector<State> &path, const Config &c,
                  std::string &reason);
struct Solution {
  bool success{false};
  std::string reason;
  std::vector<State> states;
  double elapsed_ms{}, max_violation{};
  int collision_pairs{}, variables{}, constraints{};
};
Solution solve(const Pose &ego, double steering, const std::vector<Pose> &reference,
               const std::vector<Box> &obstacles, const Config &config,
               const std::vector<State> &warm = {}, bool straight_only = false,
               bool reference_speeds = false);
struct ObservationRecovery { bool active{false}; Pose origin{}; };
// Solve and validate local mode; a stationary blind-spot failure may use a
// separately solved, fully validated short straight observation connector.
Solution solveObserved(const Grid &grid, const Pose &ego, double steering,
  const std::vector<Pose> &reference, const std::vector<Box> &obstacles,
  const Config &config, const std::vector<State> &warm = {},
  ObservationRecovery *recovery = nullptr);
} // namespace obca
