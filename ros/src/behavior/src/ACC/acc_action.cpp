#include <behavior/ACC/acc_action.h>
#include <string>
#include <mutex>

namespace Behavior
{
    ACCAction::ACCAction(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : ActionNode::ActionNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {

    }

    BT::ReturnStatus ACCAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        SpeedCmd AccCmd;
        AccCmd.state_hint = true;
        AccCmd.force_speed = true;
        AccCmd.speed = mContext->accSpeed;
        AccCmd.acceleration = mContext->accAccelerate;

        mContext->accelerationCmd["ACC"] = AccCmd;
        return BT::SUCCESS;
    }

    void ACCAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
