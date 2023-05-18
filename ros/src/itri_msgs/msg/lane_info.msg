Header header
int32 lane_id   #current lane
int32 lane_num  #total number of lane
float32 lane_width

string[] line_pos

# Right = 'R'
# Left  = 'L'
# Front = 'F'

uint32[] line_type

# EMPTY = 0
# WHITE_DASHED_LINE = 1
# WHITE_SOLID_LINE = 2
# WHITE_DOUBLE_LINE = 3
# YELLOW_DASHED_LINE
# YELLOW_SOLID_LINE
# YELLOW_DOUBLE_LINE
# RED_SOLID_LINE
# STRAIGHT_ARROW
# TURN_LEFT_ARROW
# TURN_RIGHT_ARROW
# STRAIGHT_OR_RIGHT_ARROW
# STRAIGHT_OR_LEFT_ARROW
# TURN_RIGHT_ONLY
# STOP_LINE
# LONGER_BUMP
# SHORTER_BUMP
# SLOWDOWN
# YIELD
# STOP_SIGN
# SPEED_LIMIT_20KPH
# SPEED_LIMIT_30KPH
# SPEED_LIMIT_50KPH
# SCOOTER_PARKING_PLACE
# PARKING_SPACE

float64 front_line_dis

bool[] on_route_hint
int32[] traffic_stop_line
int32[] bumper
int32[] start_right_turn
int32[] end_right_turn
int32[] start_left_turn
int32[] end_left_turn

Waypoint front_marker_point

StationInfo[] stations
string station_msg

float64 remind_distance
int32 remind_time

int32 turn_type
float64 length_to_turn
