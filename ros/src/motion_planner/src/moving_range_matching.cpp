#include <trace/utils.h>
#include <motion_planner/utils.h>
#include <motion_planner/moving_range_matching.h>

#define TRACE_TAG "MovingRangeMatching"

static const float OFF_ROAD_THRESHOLD = 10.0f;
static const int INDEX_DEFAULT = -1;

template <typename T>
static inline T Clamp(const T value, T bottom, T top)
{
    return std::max(bottom, std::min(top, value));
}

static inline float PrincipleAngleMinusPi(const float angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

MovingRangeMatching::MovingRangeMatching()
    : mIndexMatch(INDEX_DEFAULT)
    , mMatchRange(INDEX_DEFAULT)
{}

int MovingRangeMatching::RunOnce(
    const float pathResolution,
    const itri_msgs::WaypointArray & globalPath,
    const itri_msgs::CarState & carState)
{
    if (mIndexMatch == INDEX_DEFAULT)
    {
        mIndexMatch = MotionPlannerUtils::PathMatching(
            globalPath, carState.pose.pose);
    }
    else
    {
        TRACE_ASSERT_THROW(pathResolution > 0.0f);
        mMatchRange = static_cast<int>(OFF_ROAD_THRESHOLD / pathResolution);
        const auto iteratorMatch = std::next(
            globalPath.waypoints.begin(), mIndexMatch);
        const auto iteratorBegin = Clamp(std::prev(iteratorMatch, mMatchRange),
            globalPath.waypoints.begin(), globalPath.waypoints.end());
        const auto iteratorEnd = Clamp(std::next(iteratorMatch, mMatchRange),
            globalPath.waypoints.begin(), globalPath.waypoints.end());

        itri_msgs::WaypointArray globalPathPart;
        globalPathPart.waypoints.assign(iteratorBegin, iteratorEnd);
        mIndexMatch =
            MotionPlannerUtils::PathMatching(
                globalPathPart, carState.pose.pose) +
            std::distance(globalPath.waypoints.begin(), iteratorBegin);
    }

    MatchCheck(globalPath, carState);

    return mIndexMatch;
}

void MovingRangeMatching::MatchCheck(
    const itri_msgs::WaypointArray & globalPath,
    const itri_msgs::CarState & carState)
{
    const float headCheck = std::abs(PrincipleAngleMinusPi(
        globalPath.waypoints[mIndexMatch].pose.pose.orientation.z -
            carState.pose.pose.orientation.z));
    const float rangeCheck = std::hypot(
        (globalPath.waypoints[mIndexMatch].pose.pose.position.x -
            carState.pose.pose.position.x),
        (globalPath.waypoints[mIndexMatch].pose.pose.position.y -
            carState.pose.pose.position.y));

    if (rangeCheck > OFF_ROAD_THRESHOLD || headCheck > M_PI_2)
        mIndexMatch = INDEX_DEFAULT;
}
