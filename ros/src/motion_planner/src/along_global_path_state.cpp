#include <motion_planner/along_global_path_state.h>
#include <motion_planner/cost_function.h>
#include <motion_planner/idle_state.h>

static const float SCALING_FACTOR = 8.0f;
static const float THRESHOLD_SAFETY_FACTOR = 1.0f;
static const float NORM_SHIFT = 0.3f;
static const float BASE_ROLL_IN = 10.0f;

static inline float Reciprocal(const float value)
{
    return 1.0f / value;
}

static inline float Sigmoid(const float x)
{
    return Reciprocal(1.0f + std::exp(-x));
}

static inline void PointOffset(
    const float distance, itri_msgs::Waypoint & point)
{
    point.pose.pose.position.x +=
        distance * std::cos(point.pose.pose.orientation.z + M_PI_2);
    point.pose.pose.position.y +=
        distance * std::sin(point.pose.pose.orientation.z + M_PI_2);
}

static inline bool IsCurveUnderThreshold(
    const float curvature, const float offset)
{
    const float thershold = Reciprocal(
        std::abs(offset) + THRESHOLD_SAFETY_FACTOR);
    return std::signbit(curvature * offset) || std::abs(curvature) < thershold;
}

static inline int Clamp(const int value, int bottom, int top)
{
    return std::max(bottom, std::min(top, value));
}

static inline int GetNumberOfPath(const int level)
{
    return 2 * level - 1;
}

static void PathOffset(
    const itri_msgs::WaypointArray & Oldpath, const int indexLengthRollIn,
    const float maxOffset, const float bias, const std::vector<float> curvature,
    itri_msgs::WaypointArray & newPath)
{
    newPath.waypoints.clear();

    for (size_t j = 0; j < Oldpath.waypoints.size(); ++ j)
    {
        const float distNorm =
            0.8f * static_cast<float>(j) / static_cast<float>(indexLengthRollIn) -
            NORM_SHIFT;
        const float offset =
            maxOffset * Sigmoid(SCALING_FACTOR * distNorm) + bias;

        if (IsCurveUnderThreshold(curvature[j], offset))
        {
            itri_msgs::Waypoint point = Oldpath.waypoints[j];
            PointOffset(offset, point);
            newPath.waypoints.push_back(point);
        }
    }
}

static void LocalPathGenerate(
    const int indexMatch,
    const GlobalPath & globalPath,
    const itri_msgs::CarState & carState,
    const itri_msgs::plan & behaivorMsg,
    const MotionPlannerUtils::PlannerParam & plannerParam,
    const itri_msgs::WaypointArray & waypoints,
    itri_msgs::WaypointArrays & localPaths)
{
    const int indexLengthRollIn = static_cast<int>(
        plannerParam.rollInFactor * std::sqrt(carState.twist.twist.linear.x) /
        globalPath.mResolution + BASE_ROLL_IN);
    const int indexLengthRollOut = static_cast<int>(
        plannerParam.rollOutDist / globalPath.mResolution);

    if (waypoints.waypoints.size() < 2)
    {
        const int indexBegin = Clamp(
            indexMatch, 0, globalPath.mPath.waypoints.size() - 2);
        const int indexEnd = Clamp(
            indexBegin + indexLengthRollIn + indexLengthRollOut,
            0, globalPath.mPath.waypoints.size() - 2);

        const float carOffsetToPath = MotionPlannerUtils::LateralProjection(
            (carState.pose.pose.position.x -
                globalPath.mPath.waypoints[indexMatch].pose.pose.position.x),
            (carState.pose.pose.position.y -
                globalPath.mPath.waypoints[indexMatch].pose.pose.position.y),
            globalPath.mPath.waypoints[indexMatch].pose.pose.orientation.z);

        const int numberOfPath = GetNumberOfPath(behaivorMsg.num_level);

        localPaths.waypointArrays.clear();
        for (int i = 0; i < numberOfPath; ++ i)
        {
            const float maxOffset = behaivorMsg.bias - carOffsetToPath +
                behaivorMsg.dev * static_cast<float>(i - behaivorMsg.num_level + 1);

            itri_msgs::WaypointArray segment;
            segment.waypoints.assign(
                globalPath.mPath.waypoints.begin() + indexBegin,
                globalPath.mPath.waypoints.begin() + indexEnd);
            const std::vector<float> curvature(
                globalPath.mCurvature.begin() + indexBegin,
                globalPath.mCurvature.begin() + indexEnd);

            itri_msgs::WaypointArray localPath;
            PathOffset(segment, indexLengthRollIn, maxOffset, carOffsetToPath,
                curvature, localPath);

            itri_msgs::WaypointArray localPathReg;
            MotionPlannerUtils::PathRegularize(plannerParam.wayptRes,
                localPath, localPathReg);
            MotionPlannerUtils::SetPathHeadingAngle(localPathReg);

            localPaths.waypointArrays.push_back(localPathReg);
        }
    }
    else
    {
        const int lookahead = Clamp(static_cast<int>(
            1.3f * carState.twist.twist.linear.x), 3, 30);
        const int holdPosition =  MotionPlannerUtils::PathMatching(
            waypoints, carState.pose.pose);
        itri_msgs::WaypointArray remainWpt;

        remainWpt.waypoints.assign(
            waypoints.waypoints.begin() + holdPosition,
            waypoints.waypoints.begin() + Clamp(holdPosition + lookahead * 10,
                1, waypoints.waypoints.size() - 1));

        const int indexBegin = Clamp(
            indexMatch + lookahead, 0, globalPath.mPath.waypoints.size() - 2);
        const int indexEnd = Clamp(
            indexBegin + indexLengthRollIn + indexLengthRollOut,
            0, globalPath.mPath.waypoints.size() - 1);
        const float carOffsetToPath = MotionPlannerUtils::LateralProjection(
            (remainWpt.waypoints.back().pose.pose.position.x -
                globalPath.mPath.waypoints[indexBegin].pose.pose.position.x),
            (remainWpt.waypoints.back().pose.pose.position.y -
                globalPath.mPath.waypoints[indexBegin].pose.pose.position.y),
            globalPath.mPath.waypoints[indexBegin].pose.pose.orientation.z);

        const int numberOfPath = GetNumberOfPath(behaivorMsg.num_level);

        localPaths.waypointArrays.clear();
        for (int i = 0; i < numberOfPath; ++ i)
        {
            const float maxOffset = behaivorMsg.bias - carOffsetToPath +
                behaivorMsg.dev * static_cast<float>(i - behaivorMsg.num_level + 1);

            itri_msgs::WaypointArray segment;
            segment.waypoints.assign(
                globalPath.mPath.waypoints.begin() + indexBegin + 1,
                globalPath.mPath.waypoints.begin() + indexEnd + 1);
            const std::vector<float> curvature(
                globalPath.mCurvature.begin() + indexBegin + 1,
                globalPath.mCurvature.begin() + indexEnd + 1);

            itri_msgs::WaypointArray localPath;
            PathOffset(segment, indexLengthRollIn, maxOffset, carOffsetToPath,
                curvature, localPath);
            localPath.waypoints.insert(
                localPath.waypoints.begin(), remainWpt.waypoints.back());

            itri_msgs::WaypointArray localPathReg;
            MotionPlannerUtils::PathRegularize(plannerParam.wayptRes, localPath, localPathReg);
            MotionPlannerUtils::SetPathHeadingAngle(localPathReg);

            localPathReg.waypoints.erase(localPathReg.waypoints.begin());

            remainWpt.waypoints.insert(remainWpt.waypoints.end(),
                localPathReg.waypoints.begin(), localPathReg.waypoints.end());

            localPaths.waypointArrays.push_back(remainWpt);
        }
    }

}

static void WaypointsToMarker(
    const itri_msgs::WaypointArray & waypoints,
    visualization_msgs::Marker & marker)
{
    marker.points.clear();
    for (const auto & pt : waypoints.waypoints)
    {
        geometry_msgs::Point ptMarker;
        ptMarker.x = pt.pose.pose.position.x;
        ptMarker.y = pt.pose.pose.position.y;
        ptMarker.z = pt.pose.pose.position.z;
        marker.points.push_back(ptMarker);
    }
    marker.header.stamp = ros::Time::now();
}

std::shared_ptr<State> AlongGlobalPathState::Generate(
    const int indexMatch, MotionPlannerContext & node)
{
    node.mBehavior.hint = false;

    if (!std::signbit(indexMatch))
    {
        LocalPathGenerate(indexMatch, *node.mGlobalPath, node.mCarState,
            node.mBehavior, node.mPlannerParam, node.mWaypoints,
            node.mLocalPaths);

        // CostFunction::GetPathCost(node.mGlobalPath->mPath, node.mBehavior.dev,
        //     node.mCarState.twist.twist.linear.x, node.mPlannerParam,
        //     node.mObjectList, node.mLocalPaths);
    }
    return NextState(node);
}

std::shared_ptr<State> AlongGlobalPathState::NextState(
    MotionPlannerContext & node)
{
    if (node.mLocalPaths.waypointArrays.size() > 0)
    {
        PublishLocalPaths(node);
        PublishWaypoints(node);
        return std::make_shared<IdleState>();
    }
    return shared_from_this();
}

void AlongGlobalPathState::PublishLocalPaths(
    const MotionPlannerContext & node)
{
    node.mPubLocalPaths.publish(node.mLocalPaths);
}

void AlongGlobalPathState::PublishWaypoints(
    MotionPlannerContext & node)
{
    std::vector<float> cost;
    for (const auto & path : node.mLocalPaths.waypointArrays)
    {
        cost.push_back(path.cost);
    }
    const int indexSelected = std::distance(
        cost.begin(), std::min_element(cost.begin(), cost.end()));
    node.mWaypoints = node.mLocalPaths.waypointArrays[indexSelected];

    node.mPubWaypoints.publish(node.mWaypoints);
    WaypointsToMarker(node.mWaypoints, node.mWaypointMarker);
    node.mPubWaypointsRviz.publish(node.mWaypointMarker);
}
