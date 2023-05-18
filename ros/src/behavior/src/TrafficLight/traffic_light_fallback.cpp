#include <behavior/TrafficLight/TL_Left_sequence.h>
#include <behavior/TrafficLight/TL_Right_sequence.h>
#include <behavior/TrafficLight/TL_Red_sequence.h>
#include <behavior/TrafficLight/traffic_light_fallback.h>

namespace Behavior
{
    TrafficLightFallback::TrafficLightFallback(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : FallbackNode::FallbackNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<TLLeftSequence>(
            "Wait for Left light Sequence", mNodeSpeedCommand, mContext));
        AddChild(std::make_shared<TLRightSequence>(
            "Wait for Right light Sequence", mNodeSpeedCommand, mContext));
        AddChild(std::make_shared<TLRedSequence>(
            "Wait for Red light Sequence", mNodeSpeedCommand, mContext));
    }
}
