#include <waypoint_follower.h>
#include <move_forward_state.h>
#include <move_reverse_state.h>
#include <typeinfo>

static const float INTEGRATE_GAIN = 0.1f;
static const float INITIAL_VALUE_ZERO = 0.0f;
static const float SLOW_SPEED = 3.0f;
static const float ALMOST_STOP_SPEED = 10.0 / 3.6f;
static const float STEER_FILTER_FACTOR = 200.0f;

MoveForwardState::MoveForwardState(const WaypointFollowerNodeContext & node)
    : mSteerController()
    , mCumulateLateralErr()
    , mSteerCmdFilter(STEER_FILTER_FACTOR)
{
    const double stepSize = node.mRate.expectedCycleTime().toSec();
    mSteerController =
        std::make_shared<Controller>(node.mControllerFileDir + "/waypoint_follower/forward/", stepSize);
    mCumulateLateralErr = std::make_shared<Integrator>(stepSize);
}

std::shared_ptr<State> MoveForwardState::Handle(
    const WaypointFollowerNodeContext & node, float & steeringCommand)
{
    DoControl(node, steeringCommand);

    if (NeedMoveReverse(node))
        return std::make_shared<MoveReverseState>(node);
    return shared_from_this();
}

void MoveForwardState::DoControl(
    const WaypointFollowerNodeContext & node, float & steeringCmd)
{
    const int indexWaypoint = DoPathMatching(
        node.mCarState.pose.pose.position.x,
        node.mCarState.pose.pose.position.y,
        node.mWaypoints);

    const float angleError = AngleErrorToWaypoint(
        node.mCarState.pose.pose.position.x,
        node.mCarState.pose.pose.position.y,
        node.mCarState.pose.pose.orientation.z,
        node.mCarState.twist.twist.linear.x,
        indexWaypoint, node.mWaypoints,
        node.mParamPredictAheadTime, node.mParamWayptRes);

    const float yawRateEffect = YawRateEffect(
        angleError, node.mCarState.twist.twist.angular.z,
        node.mParamPredictAheadTime);

    const float lateralErr = LateralErrorToWaypoint(
        node.mCarState.pose.pose.position.x,
        node.mCarState.pose.pose.position.y,
        node.mCarState.pose.pose.orientation.z,
        node.mWaypoints.waypoints[indexWaypoint]);

    float cumulateLateralErr = INITIAL_VALUE_ZERO;
    if (node.mCarState.twist.twist.linear.x > SLOW_SPEED)
    {
        cumulateLateralErr = mCumulateLateralErr->OneStep(lateralErr);
        cumulateLateralErr /= node.mCarState.twist.twist.linear.x *
            node.mParamPredictAheadTime;
    }
    else
        mCumulateLateralErr->ResetState();

    float gainH = 0.2f * (node.mCarState.twist.twist.linear.x - ALMOST_STOP_SPEED + 1.0f);
    gainH = std::max(0.0f, std::min(0.8f, gainH));
    float gainL = 1.0f - 0.2f * (node.mCarState.twist.twist.linear.x - ALMOST_STOP_SPEED + 1.0f);
    gainL = std::max(0.2f, std::min(1.0f, gainL));

    const float totalErr =
        3.089f * angleError / node.mParamPredictAheadTime -
        node.mCarState.twist.twist.angular.z;

    const float steerHigh = mSteerController->OneStep(
        totalErr, node.mCarState.twist.twist.linear.x);
    float steerLow = 0.0f;
    if (std::abs(angleError) > 1e-4)
        steerLow = 0.63f * std::atan(3.089f * std::sin(angleError) / (0.5f * 3.0f));

    steeringCmd = gainH * steerHigh + gainL * steerLow;
    steeringCmd =  ConvertToSteeringWheelAngle(
        steeringCmd, node.mParamGearRatio, node.mParamSteerOffset);
    steeringCmd = mSteerCmdFilter.OneStep(steeringCmd);
}

bool MoveForwardState::NeedMoveReverse(const WaypointFollowerNodeContext & node)
{
    const int indexWaypoint = DoPathMatching(
        node.mCarState.pose.pose.position.x,
        node.mCarState.pose.pose.position.y,
        node.mWaypoints);
    return node.mSpeedCmd < 0;
}
