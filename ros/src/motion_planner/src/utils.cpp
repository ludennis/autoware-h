#include <motion_planner/utils.h>
#include <trace/utils.h>

#define TRACE_TAG "MotionPlannerUtils"

static const int SMOOTHING_FACTOR_H = 1;
static const int SMOOTHING_FACTOR_V = 5;

static const int SEARCH_PATH_START_INDEX = 0;
static const int SEARCH_PATH_END_INDEX = 5;
static const float RESOLUTION = 0.1;

static inline float PrincipleAngle(const float angle)
{
    float anglePrcp = std::atan2(std::sin(angle), std::cos(angle));
    if (std::signbit(anglePrcp))
        anglePrcp += 2.0f * M_PI;
    return anglePrcp;
}

static inline void PointShift(
    const float distance, const float angleH, const float angleV,
    itri_msgs::Waypoint & point)
{
    point.pose.pose.position.x += distance * std::cos(angleH);
    point.pose.pose.position.y += distance * std::sin(angleH);
    point.pose.pose.position.z += distance * std::tan(angleV);
}

static inline int Clamp(const int value, int bottom, int top)
{
    return std::max(bottom, std::min(top, value));
}

namespace MotionPlannerUtils
{

int PathMatching(
    const itri_msgs::WaypointArray & globalPath,
    const geometry_msgs::Pose & point)
{
    std::vector<float> distance(globalPath.waypoints.size(), 0.0f);
    for (size_t i = 0; i < distance.size(); i++)
    {
        const float diffX = globalPath.waypoints[i].pose.pose.position.x -
            point.position.x;
        const float diffY = globalPath.waypoints[i].pose.pose.position.y -
            point.position.y;
        distance[i] = std::hypot(diffX, diffY);
    }

    return std::distance(distance.begin(),
        std::min_element(distance.begin(), distance.end()));
}

void SetPathHeadingAngle(itri_msgs::WaypointArray & path)
{
    if (!path.waypoints.size())
        return;
    for (size_t i = 0; i < path.waypoints.size() - 1; i ++)
    {
        const float wayPtHead = std::atan2(
            path.waypoints[i + 1].pose.pose.position.y -
                path.waypoints[i].pose.pose.position.y,
            path.waypoints[i + 1].pose.pose.position.x -
                path.waypoints[i].pose.pose.position.x);
        path.waypoints[i].pose.pose.orientation.z = PrincipleAngle(wayPtHead);
    }
    if (path.waypoints.size() > 1)
        path.waypoints.back().pose.pose.orientation.z =
            path.waypoints[path.waypoints.size() - 2].pose.pose.orientation.z;
}

void PathRegularize(
    const float resolution, const itri_msgs::WaypointArray & oldPath,
    itri_msgs::WaypointArray & newPath)
{
    newPath.waypoints.clear();

    if (!oldPath.waypoints.size())
        return;
    itri_msgs::Waypoint point;
    point.pose.pose.position.x = oldPath.waypoints.front().pose.pose.position.x;
    point.pose.pose.position.y = oldPath.waypoints.front().pose.pose.position.y;
    point.pose.pose.position.z = oldPath.waypoints.front().pose.pose.position.z;
    newPath.waypoints.push_back(point);

    int indexMatch = 0;
    for (size_t i = 0; i < oldPath.waypoints.size() - 1; i += indexMatch)
    {
        itri_msgs::WaypointArray pathTmp;
        pathTmp.waypoints.assign(
            oldPath.waypoints.begin() + i, oldPath.waypoints.end());
        indexMatch = MotionPlannerUtils::PathMatching(pathTmp, point.pose.pose);

        const int indexAheadH = Clamp(
            i + SMOOTHING_FACTOR_H, 0, oldPath.waypoints.size() - 1);
        const int indexAheadV = Clamp(
            i + SMOOTHING_FACTOR_V, 0, oldPath.waypoints.size() - 1);

        const float diffAngleH = std::atan2(
            oldPath.waypoints[indexAheadH].pose.pose.position.y -
                point.pose.pose.position.y,
            oldPath.waypoints[indexAheadH].pose.pose.position.x -
                point.pose.pose.position.x);

        const float diffAngleV = std::atan2(
            oldPath.waypoints[indexAheadV].pose.pose.position.z -
                point.pose.pose.position.z,
            static_cast<float>(SMOOTHING_FACTOR_V));

        PointShift(resolution, diffAngleH, diffAngleV, point);
        newPath.waypoints.push_back(point);
    }
}

std::vector<int> Histogram(
    const std::vector<float> & array, const float bucketSize)
{
    float maxOfArray = *std::max_element(array.begin(), array.end());
    int numberOfBucket = static_cast<int>(std::floor(maxOfArray / bucketSize));
    TRACE_ASSERT_THROW(numberOfBucket > 0);

    std::vector<int> histogram(numberOfBucket + 1, 0);
    for (const auto & element : array)
    {
        int bucket = static_cast<int>(std::floor(element / bucketSize));
        histogram[bucket] += 1;
    }
    return histogram;
}

float LateralProjection(
    const float x, const float y, const float angle)
{
    return -x * std::sin(angle) + y * std::cos(angle);
}

float LongitudinalProjection(
    const float x, const float y, const float angle)
{
    return x * std::cos(angle) + y * std::sin(angle);
}

} // namespace MotionPlannerUtils
