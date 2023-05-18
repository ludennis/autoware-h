#include <behavior/ACC/acc_sequence.h>
#include <behavior/ACC/acc_condition.h>
#include <sequence_node.h>
#include <behavior/ACC/acc_action.h>

namespace Behavior
{
    ACCSequence::ACCSequence(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : SequenceNode::SequenceNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {
        AddChild(std::make_shared<ACCCondition>(
            "FrontCar", mContext));
        AddChild(std::make_shared<ACCAction>(
            "ACC start", mNodeSpeedCommand, mContext));
    }
}
