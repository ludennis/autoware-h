#include <behavior/TrafficLight/traffic_light_action.h>
#include <behavior/TrafficLight/TL_Red_condition.h>
#include <behavior/TrafficLight/TL_Red_sequence.h>
#include <sequence_node.h>

namespace Behavior
{
    TLRedSequence::TLRedSequence(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : SequenceNode::SequenceNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<TLRedCondition>(
            "Traffic Light Red condition", mContext));
        AddChild(std::make_shared<TrafficLightAction>(
            "Traffic Light Red", mNodeSpeedCommand, mContext));
    }
}
