#ifndef __BEHAVIOR_LANE_TYPE_H__
#define __BEHAVIOR_LANE_TYPE_H__

namespace Behavior {

    enum class LaneType {
        EMPTY = 0,
        WHITE_DASHED_LINE,
        WHITE_SOLID_LINE,
        WHITE_DOUBLE_LINE,
        YELLOW_DASHED_LINE,
        YELLOW_SOLID_LINE,
        YELLOW_DOUBLE_LINE,
        RED_SOLID_LINE,
        STRAIGHT_ARROW,
        TURN_LEFT_ARROW,
        TURN_RIGHT_ARROW,
        STRAIGHT_OR_LEFT_ARROW,
        STRAIGHT_OR_RIGHT_ARROW,
        TURN_RIGHT_ONLY,
        STOP_LINE,
        LONGER_BUMP,
        SHORTER_BUMP,
        SLOWDOWN,
        YIELD,
        STOP_SIGN,
        SPEED_LIMIT_20KPH,
        SPEED_LIMIT_30KPH,
        SPEED_LIMIT_50KPH,
        SCOOTER_PARKING_PLACE,
        PARKING_SPACE
    };

} // namespace Behavior

#endif // __BEHAVIOR_LANE_TYPE_H__
