#include <behavior/Cruise/speed_limit_action.h>
#include <string>
#include <mutex>

namespace Behavior
{
    static const float ACCELERATION = 0.5f;
    static const float DECELERATION = -1.0f;
    SpeedLimitAction::SpeedLimitAction(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : ActionNode::ActionNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {

    }

    BT::ReturnStatus SpeedLimitAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        SpeedCmd SpeedLimitCmd;
        float acceleration = (mContext->vehPosSD.speed < mContext->maxSpeed ?
            ACCELERATION : DECELERATION);
        SpeedLimitCmd.state_hint = true;
        SpeedLimitCmd.speed = mContext->maxSpeed;
        SpeedLimitCmd.acceleration = acceleration;

        mContext->accelerationCmd["SPEEDUP"] = SpeedLimitCmd;
        return BT::SUCCESS;
    }

    void SpeedLimitAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
