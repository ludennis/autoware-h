#include <behavior/avoidance_sequence.h>
#include <string>
#include <control_node.h>
#include <fallback_node.h>
#include <behavior/obj_on_path_condition.h>
#include <behavior/genbias_condition.h>
#include <behavior/turn_out_action.h>
#include <behavior/passing_action.h>
#include <behavior/turn_in_action.h>
#include <behavior/aeb_triggered_action.h>

namespace Behavior
{
    const float AvoidanceSequence::OBJECTS_SEARCHING_RANGE = 40.0f;
    const float AvoidanceSequence::MIN_CAR_SPEED = 8.0f / 3.6f;
    const float AvoidanceSequence::DEFAULT_ACCELERATION = 1.0f;
    const float AvoidanceSequence::STOP_SPEED = 1.0f / 3.6f;

    AvoidanceSequence::AvoidanceSequence(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : SequenceNodeWithMemory::SequenceNodeWithMemory(name)
        , mNodeCommand(std::make_shared<BehaviorOutput>())
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mTargetPoint(0.0f)
        , mTargetD(0.0f)
        , mBiasResolution(10.0f)
        , mWaitingTime(0.0f)
        , mObstacleStayTime(0.0)
        , mFrontObstacle({})
        , mInterruptTime(0.0)
        , mSpeedCommand({})
    {
        const auto avoidanceFallback =
            std::make_shared<BT::FallbackNode>("Avoidance Actions");
        const auto normalSequence =
            std::make_shared<BT::SequenceNodeWithMemory>("Normal Action");
        const auto turnOutFallback =
            std::make_shared<BT::FallbackNode>("TurnOut Fallback");
        const auto passingFallback =
            std::make_shared<BT::FallbackNode>("Passing Fallback");

        turnOutFallback->AddChild(std::make_shared<AebTriggeredAction>(
            "Aeb in TurnOut", mContext));
        turnOutFallback->AddChild(std::make_shared<TurnOutAction>(
            "TurnOut", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        passingFallback->AddChild(std::make_shared<AebTriggeredAction>(
            "Aeb in Passing", mContext));
        passingFallback->AddChild(std::make_shared<PassingAction>(
            "Passing", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        normalSequence->AddChild(turnOutFallback);
        normalSequence->AddChild(passingFallback);

        avoidanceFallback->AddChild(normalSequence);
        avoidanceFallback->AddChild(std::make_shared<TurnInAction>(
            "Interrupted", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        AddChild(std::make_shared<ObjOnPathCondition>(
            "ObjOnPath", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        AddChild(std::make_shared<GenBiasCondition>(
            "GenBias", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        AddChild(avoidanceFallback);
    }

    void AvoidanceSequence::EnsureMinDrivingSpeed()
    {
        if (mContext->vehPosSD.speed < AvoidanceSequence::STOP_SPEED)
        {
            mNodeSpeedCommand->state_hint = true;
            mNodeSpeedCommand->speed = AvoidanceSequence::MIN_CAR_SPEED;
            mNodeSpeedCommand->acceleration = AvoidanceSequence::DEFAULT_ACCELERATION;
            mNodeSpeedCommand->force_speed = false;
        }
        else
        {
            mNodeSpeedCommand->state_hint = false;
        }
    }
}
