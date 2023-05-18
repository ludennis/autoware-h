#include <behavior/Cruise/cruise_fallback.h>
#include <behavior/Cruise/speed_limit_action.h>
#include <behavior/Cruise/bumper_condition.h>
#include <behavior/Cruise/bumper_action.h>
#include <behavior/Cruise/curve_condition.h>
#include <behavior/Cruise/curve_action.h>
#include <string>

namespace Behavior
{
    CruiseFallback::CruiseFallback(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : FallbackNode::FallbackNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        const auto BumperSpeedSequence =
            std::make_shared<BT::SequenceNode>("Bumper Sequence");
        const auto CurveSpeedSequence =
            std::make_shared<BT::SequenceNode>("Curve Sequence");

        BumperSpeedSequence->AddChild(std::make_shared<BumperCondition>(
            "Bumper condition", mContext));
        BumperSpeedSequence->AddChild(std::make_shared<BumperAction>(
            "Bumper", mNodeSpeedCommand, mContext));

        CurveSpeedSequence->AddChild(std::make_shared<CurveCondition>(
            "Curve condition", mContext));
        CurveSpeedSequence->AddChild(std::make_shared<CurveAction>(
            "Curve", mNodeSpeedCommand, mContext));

        AddChild(BumperSpeedSequence);
        AddChild(CurveSpeedSequence);
        AddChild(std::make_shared<SpeedLimitAction>(
            "Speedup", mNodeSpeedCommand, mContext));
    }
}
