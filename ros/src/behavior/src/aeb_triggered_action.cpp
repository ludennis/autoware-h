#include <behavior/aeb_triggered_action.h>
#include <string>
#include <mutex>

namespace Behavior
{
    AebTriggeredAction::AebTriggeredAction(
        std::string name,
        const std::shared_ptr<Context> & context)
        : ActionNode::ActionNode(name)
        , mContext(context)
    {

    }

    BT::ReturnStatus AebTriggeredAction::Tick()
    {
        while (mContext->aebCommand.state_hint)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::lock_guard<std::mutex> guard(mContext->mutex);

            ROS_INFO_STREAM("The Action " << get_name() << " is doing some operations");
            if (is_halted())
            {
                ROS_INFO_STREAM("The Action " << get_name() << " is halted");
                return BT::HALTED;
            }
        }

        if(!mContext->aebCommand.state_hint)
        {
            ROS_INFO_STREAM("The Action " << get_name() << " is failed");
            return BT::FAILURE;
        }
    }

    void AebTriggeredAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
