#include <motion_planner/cost_function.h>
#include <trace/utils.h>

#define TRACE_TAG "CostFunction"

static const float INITIAL_ZERO = 0.0f;
static const float GAUSSIAN_VARIANCE = 1.0f;
static const float PADDING_VALUE = 1.0f;
static const float CLEARANCE = 0.5f;
static const float RANGE_TIME_CONSTANT = 3.0f;
static const float EXTEND_RANGE_CONSTANT = 1.0f;
static const float RANGE_BASE = 10.0f;

static inline void SoftMax(std::vector<float> & array)
{
    for (auto & element : array)
        element = std::exp(element);

    const float sum = std::accumulate(array.begin(), array.end(), INITIAL_ZERO);
    for (auto & element : array)
        element /= sum;
}

static inline std::vector<float> Gaussian(const int length)
{
    std::vector<float> array(length, INITIAL_ZERO);
    for (int i = 0; i < length; ++ i)
    {
        array[i] =
            std::exp(-0.5f * std::pow(
                static_cast<float>(i - length / 2) / GAUSSIAN_VARIANCE, 2.0f)) /
            std::sqrt(2.0f * M_PI) / GAUSSIAN_VARIANCE;
    }

    return array;
}

std::vector<float> CostFunction::Convolution(
    const std::vector<float> & kernal,
    const std::vector<float> & target)
{
    std::vector<float> kernalFlip(kernal.begin(), kernal.end());
    std::reverse(kernalFlip.begin(),kernalFlip.end());

    std::vector<float> result(target.size(), INITIAL_ZERO);
    for (int i = 0; i < result.size(); ++ i)
    {
        for (int j = 0; j < kernal.size(); ++ j)
        {
            const int index = j + i - static_cast<int>(result.size()) / 2;
            if (index >= 0 && index < target.size())
                result[i] += kernal[j] * target[index];
            else
                result[i] += kernal[j] * PADDING_VALUE;
        }
    }

    return result;
}

float CostFunction::FrenetTransform(
    const itri_msgs::WaypointArray & path,
    itri_msgs::DetectedObjectArray & objectList)
{
    std::vector<float> accumulateDist(path.waypoints.size(), INITIAL_ZERO);
    for (size_t i = 0; i < path.waypoints.size() - 1; ++ i)
    {
        accumulateDist[i + 1] = accumulateDist[i] + std::hypot(
            path.waypoints[i + 1].pose.pose.position.x -
                path.waypoints[i].pose.pose.position.x,
            path.waypoints[i + 1].pose.pose.position.y -
                path.waypoints[i].pose.pose.position.y);
    }

    for (auto & object : objectList.objects)
    {
        for (auto & point : object.convex_hull.polygon.points)
        {
            geometry_msgs::Pose pointXY;
            pointXY.position.x = point.x;
            pointXY.position.y = point.y;
            const int closestIndex = MotionPlannerUtils::PathMatching(
                path, pointXY);

            point.d = MotionPlannerUtils::LateralProjection(
                point.x -
                    path.waypoints[closestIndex].pose.pose.position.x,
                point.y -
                    path.waypoints[closestIndex].pose.pose.position.y,
                path.waypoints[closestIndex].pose.pose.orientation.z);

            point.s = accumulateDist[closestIndex] +
                MotionPlannerUtils::LongitudinalProjection(
                    point.x -
                        path.waypoints[closestIndex].pose.pose.position.x,
                    point.y -
                        path.waypoints[closestIndex].pose.pose.position.y,
                    path.waypoints[closestIndex].pose.pose.orientation.z);
        }
    }

    return accumulateDist.back();
}

void CostFunction::FindDangerObjects(
    const itri_msgs::WaypointArray & path,
    const float boundary, const float range, const float extendRange,
    itri_msgs::DetectedObjectArray & objectList,
    itri_msgs::DetectedObjectArray & dangerObjects)
{
    const float pathS = CostFunction::FrenetTransform(path, objectList);
    std::vector<int> dangerIndex;
    std::vector<float> dangerObjectS;
    for (int i = 0; i < objectList.objects.size(); ++ i)
    {
        std::vector<float> dangerPointS;
        for (const auto & point :
                objectList.objects[i].convex_hull.polygon.points)
        {
            if (std::abs(point.d) < boundary &&
                !std::signbit(point.s) && point.s < pathS)
                dangerPointS.push_back(point.s);
        }
        if (dangerPointS.size() > 0)
        {
            dangerIndex.push_back(i);
            dangerObjectS.push_back(
                *std::min_element(dangerPointS.begin(), dangerPointS.end()));
        }
    }

    dangerObjects.objects.clear();
    if (dangerIndex.size() > 0)
    {
        const float searchRange = extendRange + std::min(range,
            *std::min_element(dangerObjectS.begin(), dangerObjectS.end()));
        for (size_t i = 0; i < dangerObjectS.size(); ++ i)
        {
            if (dangerObjectS[i] < searchRange)
                dangerObjects.objects.push_back(
                    objectList.objects[dangerIndex[i]]);
        }
    }
}

bool CostFunction::HaveObjectOnPath(
    const itri_msgs::WaypointArray & path,
    itri_msgs::DetectedObjectArray & objectList)
{
    const float pathS = CostFunction::FrenetTransform(path, objectList);
    bool result = false;
    for (const auto & object : objectList.objects)
    {
        for (const auto & point : object.convex_hull.polygon.points)
        {
            result = result || (
                std::abs(point.d) < CLEARANCE &&
                !std::signbit(point.s) && point.s < pathS);
        }
    }
    return result;
}

std::vector<float> CostFunction::CollisionCheck(
    const itri_msgs::WaypointArrays & localPaths,
    const float boundary, const float range, const float extendRange,
    itri_msgs::DetectedObjectArray & objectList)
{
    std::vector<float> result(localPaths.waypointArrays.size(), INITIAL_ZERO);

    const auto middle = std::next(
        localPaths.waypointArrays.begin(),
        localPaths.waypointArrays.size() / 2);

    itri_msgs::DetectedObjectArray dangerObjects;
    CostFunction::FindDangerObjects(
        *middle, boundary, range, extendRange, objectList, dangerObjects);

    for (size_t i = 0; i < localPaths.waypointArrays.size(); ++ i)
    {
        const bool collisionCheck = CostFunction::HaveObjectOnPath(
            localPaths.waypointArrays[i], dangerObjects);
        result[i] = static_cast<float>(collisionCheck);
    }

    return result;
}

std::vector<float> CostFunction::CollisionCost(
    const itri_msgs::WaypointArrays & localPaths,
    const float pathDev, const float carSpeed,
    itri_msgs::DetectedObjectArray & objectList)
{
    const float boundary = CLEARANCE +
        pathDev * static_cast<float>(localPaths.waypointArrays.size() / 2);
    const float range = RANGE_TIME_CONSTANT * carSpeed + RANGE_BASE;
    const float extendRange = EXTEND_RANGE_CONSTANT * carSpeed;
    std::vector<float> collisionCheck = CostFunction::CollisionCheck(
        localPaths, boundary, range, extendRange, objectList);
    std::vector<float> kernal = Gaussian(collisionCheck.size());
    std::vector<float> collisionCost = CostFunction::Convolution(
        kernal, collisionCheck);
    SoftMax(collisionCost);

    return collisionCost;
}

std::vector<float> CostFunction::TransitionCost(
    const itri_msgs::WaypointArrays & localPaths,
    const itri_msgs::WaypointArray & globalPath)
{
    std::vector<float>transitionCost;
    for (const auto & path : localPaths.waypointArrays)
    {
        const int frontIndex = MotionPlannerUtils::PathMatching(
            globalPath, path.waypoints.front().pose.pose);
        const int backIndex = MotionPlannerUtils::PathMatching(
            globalPath, path.waypoints.back().pose.pose);
        const float lateralDisplacement =
            MotionPlannerUtils::LateralProjection(
                path.waypoints.back().pose.pose.position.x -
                    globalPath.waypoints[backIndex].pose.pose.position.x,
                path.waypoints.back().pose.pose.position.y -
                    globalPath.waypoints[backIndex].pose.pose.position.y,
                globalPath.waypoints[backIndex].pose.pose.orientation.z) -
            MotionPlannerUtils::LateralProjection(
                path.waypoints.front().pose.pose.position.x -
                    globalPath.waypoints[frontIndex].pose.pose.position.x,
                path.waypoints.front().pose.pose.position.y -
                    globalPath.waypoints[frontIndex].pose.pose.position.y,
                globalPath.waypoints[frontIndex].pose.pose.orientation.z);

        transitionCost.push_back(std::abs(lateralDisplacement));
    }
    SoftMax(transitionCost);

    return transitionCost;
}

void CostFunction::GetPathCost(
    const itri_msgs::WaypointArray & globalPath,
    const float pathDev, const float carSpeed,
    const MotionPlannerUtils::PlannerParam & param,
    itri_msgs::DetectedObjectArray & objectList,
    itri_msgs::WaypointArrays & localPaths)
{
    TRACE_ASSERT_THROW(localPaths.waypointArrays.size() > 0);
    const std::vector<float> collisionCost = CostFunction::CollisionCost(
        localPaths, pathDev, carSpeed, objectList);
    const std::vector<float> transitionCost = CostFunction::TransitionCost(
        localPaths, globalPath);

    for (size_t i = 0; i < localPaths.waypointArrays.size(); ++ i)
    {
        localPaths.waypointArrays[i].cost =
            param.weightCollision * collisionCost[i] +
            param.weightTransition * transitionCost[i];
    }
}
