std_msgs/Header header

# Legacy diagnostic triple. Kept first and unrenamed so existing consumers/plots keep working.
float64 gap_diff
float64 vs_diff
float64 gap_int

# Which term produced speed_limit. Mirrors f1tenth_control's CRUISE_CONSTRAINT_* constants.
uint8 CONSTRAINT_FEEDBACK=0
uint8 CONSTRAINT_BRAKING=1
uint8 CONSTRAINT_EMERGENCY=2
uint8 CONSTRAINT_MAX_SPEED=3
uint8 CONSTRAINT_NO_ACCEL=4
uint8 active_constraint

# Gap chain, so a tuning session can separate a measurement problem from a policy problem:
# raw_gap -> (minus uncertainty_sigma*sigma_gap) -> effective_gap, compared against desired_gap.
float64 raw_gap
float64 effective_gap
float64 desired_gap
float64 sigma_gap
float64 horizon_tau

# Speed chain. speed_limit is min(feedback_speed, braking_speed) after clamping.
float64 ego_speed
float64 opponent_speed
float64 feedback_speed
float64 braking_speed
float64 speed_limit
