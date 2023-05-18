#ifndef __MOVING_RANGE_MATCHING_H__
#define __MOVING_RANGE_MATCHING_H__

#include <itri_msgs/CarState.h>
#include <itri_msgs/WaypointArray.h>

class MovingRangeMatching
{
public:
    MovingRangeMatching();

    int RunOnce(
        const float pathResolution,
        const itri_msgs::WaypointArray & globalPath,
        const itri_msgs::CarState & carState);

protected:
    void MatchCheck(
        const itri_msgs::WaypointArray & globalPath,
        const itri_msgs::CarState & carState);

private:
    int mIndexMatch;
    int mMatchRange;
};

#endif // __MOVING_RANGE_MATCHING_H__
