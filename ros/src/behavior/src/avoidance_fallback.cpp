#include <behavior/avoidance_fallback.h>
#include <string>
#include <control_node.h>
#include <fallback_node.h>
#include <behavior/avoidance_sequence.h>
#include <behavior/slow_down_sequence.h>

namespace Behavior
{
    AvoidanceFallback::AvoidanceFallback(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : FallbackNode::FallbackNode(name)
        , mNodeCommand(std::make_shared<BehaviorOutput>())
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<AvoidanceSequence>(
            "Avoidance Sequence", mNodeSpeedCommand, mContext));
        AddChild(std::make_shared<SlowDownSequence>(
            "SlowDown Sequence", mNodeSpeedCommand, mContext));
    }
}
