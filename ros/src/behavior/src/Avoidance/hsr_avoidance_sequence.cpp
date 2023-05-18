#include <string>
#include <control_node.h>
#include <fallback_node.h>
#include <behavior/Avoidance/hsr_check_lane_condition.h>
#include <behavior/Avoidance/hsr_obstacle_on_roadside_condition.h>
#include <behavior/Avoidance/hsr_gen_bias_action.h>
#include <behavior/Avoidance/hsr_slow_down_action.h>
#include <behavior/Avoidance/hsr_reset_bias_action.h>
#include <behavior/Avoidance/hsr_oa_trun_signal.h>

namespace Behavior
{
    hsrAvoidanceSequence::hsrAvoidanceSequence(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : SequenceNode::SequenceNode(name)
        , mNodeCommand(std::make_shared<BehaviorOutput>())
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mCheckRightArea(false)
        , mCheckLeftArea(false)
        , mSideObstacles({})
        , mOnRoadObstacles({})
        , mBias(0.0f)
    {
        const auto avoidanceActionFallback =
            std::make_shared<BT::FallbackNode>("Avoidance Actions");

        const auto avoidanceActionSequence =
            std::make_shared<BT::SequenceNode>("Avoidance Sequence");

        avoidanceActionSequence->AddChild(std::make_shared<checkLaneCondition>(
            "ONSIDELANE", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        avoidanceActionSequence->AddChild(std::make_shared<objOnRoadsideCondition>(
            "OBJONROAD", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        avoidanceActionSequence->AddChild(std::make_shared<genBiasAction>(
            "GENBIAS", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        avoidanceActionFallback->AddChild(avoidanceActionSequence);
        avoidanceActionFallback->AddChild(std::make_shared<resetBiasAction>(
            "RESETBIAS", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        AddChild(std::make_shared<hsrSlowDownAction>(
             "SLOWDOWN", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        AddChild(std::make_shared<hsrOATurnSignalAction>(
            "OA_TURN_SIGNAL", mNodeCommand, mNodeSpeedCommand, mContext, *this));

        AddChild(avoidanceActionFallback);

        // avoidanceActionSFallback->AddChild(std::make_shared<genBiasAction>(
        //     "GENBIAS", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        // avoidanceActionSFallback->AddChild(std::make_shared<hsrSlowDownAction>(
        //     "SLOWDOWN", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        //
        // AddChild(std::make_shared<checkLaneCondition>(
        //     "ONSIDELANE", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        // AddChild(std::make_shared<objOnRoadsideCondition>(
        //     "OBJONROAD", mNodeCommand, mNodeSpeedCommand, mContext, *this));
        // AddChild(avoidanceActionSFallback);
    }
}
