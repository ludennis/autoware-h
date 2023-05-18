#include <behavior/TrafficLight/traffic_light_action.h>
#include <behavior/TrafficLight/TL_Right_condition.h>
#include <behavior/TrafficLight/TL_Right_sequence.h>
#include <sequence_node.h>

namespace Behavior
{
    TLRightSequence::TLRightSequence(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : SequenceNode::SequenceNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<TLRightCondition>(
            "Traffic Light Right condition", mContext));
        AddChild(std::make_shared<TrafficLightAction>(
            "Traffic Light Right", mNodeSpeedCommand, mContext));
    }
}
