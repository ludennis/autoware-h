#include <behavior/TrafficLight/traffic_light_action.h>
#include <behavior/TrafficLight/TL_Left_condition.h>
#include <behavior/TrafficLight/TL_Left_sequence.h>
#include <sequence_node.h>

namespace Behavior
{
    TLLeftSequence::TLLeftSequence(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : SequenceNode::SequenceNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<TLLeftCondition>(
            "Traffic Light Left condition", mContext));
        AddChild(std::make_shared<TrafficLightAction>(
            "Traffic Light Left", mNodeSpeedCommand, mContext));
    }
}
