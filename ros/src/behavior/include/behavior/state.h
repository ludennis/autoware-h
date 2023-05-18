#ifndef __BEHAVE_STATE_H__
#define __BEHAVE_STATE_H__

namespace Behavior
{
    #define BEHAVIOR_STATES \
        BS(INITIAL) \
        BS(AEB) \
        BS(LANE_FOLLOW) \
        BS(LANE_CHANGE) \
        BS(AVOIDANCE) \
        BS(STOP) \
        BS(PARKING) \
        BS(EMERGENCY) \
        BS(FINISH) \
        BS(ACC) \
        BS(BUMP) \
        BS(INTERSECTION) \
        BS(TRAFFIC_LIGHT_RED) \
        BS(TRAFFIC_LIGHT_WAIT_FOR_TURN_LEFT) \
        BS(TRAFFIC_LIGHT_WAIT_FOR_TURN_RIGHT) \

    enum class State
    {
    #define BS(name) name,
        BEHAVIOR_STATES
    #undef BS
    };

    const char * ToString(const State);

} // namespace Behavior

#endif // __BEHAVE_STATE_H__
