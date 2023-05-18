#include <utils.h>
#include <global_path.h>

static const float BUCKET_SIZE = 0.01f;
static const float HALF = 0.5f;
static const float RESOLUTION_DEFAULT = 1.0f;

static inline float PrincipleAngleMinusPi(const float angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

static inline float SupplementaryAngle(const float angle)
{
    return PrincipleAngleMinusPi(M_PI - angle);
}

GlobalPath::GlobalPath(const itri_msgs::Path & path)
    : mPath()
    , mCurvature()
    , mResolution(RESOLUTION_DEFAULT)
{
    mPath.waypoints.assign(path.waypoints.begin(), path.waypoints.end());
    mCurvature.resize(mPath.waypoints.size(), 0.0f);
    CalculatePathResolution();
    CalculatePathCurvature();
}

void GlobalPath::CalculatePathResolution()
{
    std::vector<float> distance(mPath.waypoints.size() - 1, 0.0f);
    for (size_t i = 0; i < distance.size(); i ++)
    {
        distance[i] = std::hypot(
            mPath.waypoints[i + 1].pose.pose.position.x -
            mPath.waypoints[i].pose.pose.position.x,
            mPath.waypoints[i + 1].pose.pose.position.y -
            mPath.waypoints[i].pose.pose.position.y);
    }

    const std::vector<int> distanceHist =
        MotionPlannerUtils::Histogram(distance, BUCKET_SIZE);
    const float maxHist = std::distance(
        distanceHist.begin(), std::max_element(
            distanceHist.begin(), distanceHist.end()));
    mResolution = maxHist * BUCKET_SIZE;
}

void GlobalPath::CalculatePathCurvature()
{
    for (size_t i = 1; i < mCurvature.size(); i ++)
    {
        const float angleHalf = HALF * SupplementaryAngle(
            mPath.waypoints[i].pose.pose.orientation.z -
            mPath.waypoints[i - 1].pose.pose.orientation.z);
        const float radius = HALF * mResolution * std::tan(angleHalf);
        mCurvature[i] = std::pow(radius, -1.0f);
    }
}
