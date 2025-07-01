std_msgs/Header header

float64 timestamp

float32 loc_x
float32 loc_y
float32 orientation

float32[] particles_x
float32[] particles_y
float32[] particles_weight
float32[] locations_weight

uint16 current_navcommand
float32 obs_avdir_x
float32 obs_avdir_y
bool path_blocked

float32 battery_voltage
bool emergency_stop

# Cobot Operation modes:
# 0 = JoystickMode
# 1 = JoystickObstAvMode
# 2 = MotionPrimitiveMode
# 3 = MotionPrimitiveObstAvMode
# 4 = TeleOpMode
# 5 = TestMode
# 6 = AllStopMode
int16 cobot_mode

# Cobot Commands:
# 0 = NavCmdNone
# 1 = NavCmdGoTo
# 2 = NavCmdMoveStraight
# 3 = NavCmdMoveDownCorridor
# 4 = NavCmdIntegratedTurn
# 5 = NavCmdInPlaceTurn
# 6 = NavCmdTurnAndMoveStraight
# 7 = NavCmdAbort
int16 current_command
bool nav_complete

int32 current_edge
float32 edge_progress
float32 edge_remaining
float64 edge_navduration

float32[] path_plan_x
float32[] path_plan_y
