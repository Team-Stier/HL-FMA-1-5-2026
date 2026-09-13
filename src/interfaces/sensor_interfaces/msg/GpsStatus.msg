std_msgs/Header header

uint8 SOLUTION_NO_FIX=0
uint8 SOLUTION_2D=1
uint8 SOLUTION_3D=2
uint8 SOLUTION_DGNSS=3
uint8 SOLUTION_RTK_FLOAT=4
uint8 SOLUTION_RTK_FIXED=5
uint8 SOLUTION_DEAD_RECKONING=6
uint8 SOLUTION_GNSS_DEAD_RECKONING=7
uint8 SOLUTION_TIME_ONLY=8

uint8 solution
string status_text
bool fix_ok
uint8 fix_type
bool differential_solution

uint8 CARRIER_NONE=0
uint8 CARRIER_FLOAT=1
uint8 CARRIER_FIXED=2

uint8 carrier_solution

uint8 SPOOF_UNKNOWN=0
uint8 SPOOF_NONE=1
uint8 SPOOF_INDICATED=2
uint8 SPOOF_MULTIPLE=3

bool nav_status_available
uint8 spoofing_state
uint32 time_to_first_fix_ms

uint8 satellites_used
float64 horizontal_accuracy_m
float64 vertical_accuracy_m
float64 position_dop
float64 speed_accuracy_mps
float64 heading_accuracy_deg
