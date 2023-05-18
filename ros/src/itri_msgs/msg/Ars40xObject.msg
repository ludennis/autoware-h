# while persists in consecutive messages - it is the same object, if disappears and appears again - new one
uint8 id

float32 width
float32 length

geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration

# dBm^2
float32 radar_cross_section

uint8 POINT = 0
uint8 CAR = 1
uint8 TRUCK = 2
uint8 MOTORCYCLE = 4
uint8 BICYCLE = 5
uint8 WIDE = 6
uint8 Class

uint8 MOVING = 0
uint8 STATIONARY = 1
uint8 ONCOMING = 2
uint8 STATIONARY_CANDIDATE = 3
uint8 UNKNOWN = 4
uint8 CROSSING_STATIONARY = 5
uint8 CROSSING_MOVING = 6
uint8 STOPPED = 7
uint8 dynamic

uint8 DELETED = 0
uint8 NEW_CREATED = 1
uint8 MEASURED = 2
uint8 PREDICTED = 3
uint8 DELETED_FOR_MERGE = 4
uint8 NEW_FROM_MERGE = 5
uint8 measurment

# probability of existence, percentage
float32 probability