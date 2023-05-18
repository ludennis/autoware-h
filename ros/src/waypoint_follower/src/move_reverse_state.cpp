#include <waypoint_follower.h>
#include <move_forward_state.h>
#include <move_reverse_state.h>
#include <cmath>
#include <typeinfo>

static const float INTEGRATE_GAIN = 0.0f;
static const float INITIAL_VALUE_ZERO = 0.0f;
static const float SLOW_SPEED = 3.0f;
static const float ALMOST_STOP_SPEED = 0.0f / 3.6f;

static inline float GetPrincipalAngle(const float & angle)
{
    float angleMod = std::fmod(angle, 2.0f * M_PI);
    if (std::signbit(angleMod))
        angleMod += 2.0f * M_PI;
    return angleMod;
}

MoveReverseState::MoveReverseState(const WaypointFollowerNodeContext & node)
    : mSteerController()
    , mSteerCmdFilter(100.0f)
{
    const double stepSize = node.mRate.expectedCycleTime().toSec();
    mSteerController =
        std::make_shared<Controller>(node.mControllerFileDir + "/waypoint_follower/reverse/", stepSize);
}

std::shared_ptr<State> MoveReverseState::Handle(
    const WaypointFollowerNodeContext & node, float & steeringCommand)
{
    DoControl(node, steeringCommand);

    if (NeedMoveForward(node))
        return std::make_shared<MoveForwardState>(node);
    return shared_from_this();
}

void MoveReverseState::DoControl(
    const WaypointFollowerNodeContext & node, float & steeringCmd)
{
    const float reverseHead = GetPrincipalAngle(
        node.mCarState.pose.pose.orientation.z + M_PI);
    const float reverseX = node.mCarState.pose.pose.position.x + 1.0f * std::cos(reverseHead);
    const float reverseY = node.mCarState.pose.pose.position.y + 1.0f * std::sin(reverseHead);

    const int indexWaypoint = DoPathMatching(
        reverseX, reverseY, node.mWaypoints);

    const float paramPredictAheadTime = 0.8f;
    const float yawPredictAheadTime = 0.0f;

    const float angleError = AngleErrorToWaypoint(
        reverseX,
        reverseY,
        reverseHead,
        node.mCarState.twist.twist.linear.x,
        indexWaypoint, node.mWaypoints,
        paramPredictAheadTime, node.mParamWayptRes);

    const float yawRateEffect = YawRateEffect(
        angleError, node.mCarState.twist.twist.angular.z,
        yawPredictAheadTime);

    const float totalErr = angleError - yawRateEffect;

    if (indexWaypoint >= std::max(static_cast<int>(node.mWaypoints.waypoints.size()) - 40, 0))
    {
        steeringCmd -= 0.04f * steeringCmd;
        std::cout << "/* message */" << '\n';
    }
    else
    {
        if (node.mCarState.twist.twist.linear.x > ALMOST_STOP_SPEED)
        {
            steeringCmd = mSteerCmdFilter.OneStep(mSteerController->OneStep(
                totalErr, node.mCarState.twist.twist.linear.x));
            steeringCmd =  ConvertToSteeringWheelAngle(
                steeringCmd, node.mParamGearRatio, node.mParamSteerOffset);
        }
        else
        {
            if (std::abs(angleError) > 1e-4)
                steeringCmd = 2.0f * std::atan(2.612f * std::sin(-angleError) / (0.5f * 4.0f));
            else
                steeringCmd = 0.0f;
            steeringCmd =  ConvertToSteeringWheelAngle(
                steeringCmd, node.mParamGearRatio, node.mParamSteerOffset);
            steeringCmd = mSteerCmdFilter.OneStep(steeringCmd);
        }
    }
}

bool MoveReverseState::NeedMoveForward(const WaypointFollowerNodeContext & node)
{
    const int indexWaypoint = DoPathMatching(
        node.mCarState.pose.pose.position.x,
        node.mCarState.pose.pose.position.y,
        node.mWaypoints);
    return node.mSpeedCmd >= 0;
}
