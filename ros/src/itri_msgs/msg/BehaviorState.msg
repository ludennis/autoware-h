int32 behavior_state
string behavior_stete_string

uint8 INITIAL = 0
uint8 AEB = 1
uint8 LANE_FOLLOW = 2
uint8 LANE_CHANGE = 3
uint8 AVOIDANCE = 4
uint8 STOP = 5
uint8 PARKING = 6
uint8 EMERGENCY = 7
uint8 FINISH = 8
uint8 ACC = 9
uint8 BUMP = 10
uint8 INTERSECTION = 11
uint8 TRAFFIC_LIGHT_RED = 12
uint8 TRAFFIC_LIGHT_WAIT_FOR_TURN_LEFT = 13
uint8 TRAFFIC_LIGHT_WAIT_FOR_TURN_RIGHT = 14
uint8 CURVE = 15

int32 obj_id
bool obstacles
bool avoidance
