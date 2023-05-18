#include <behavior/lane_following.h>
#include <behavior/Cruise/curve_action.h>
#include <string>
#include <mutex>

namespace Behavior
{
    static const float SPEEDUP_CONST = 0.5f / 3.6f; // m/s

    CurveAction::CurveAction(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : ActionNode::ActionNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {

    }

    BT::ReturnStatus CurveAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        LaneFollowing LF;

        SpeedCmd CurveCmd;
        LF.CurveSpeed(mContext->vehPosSD.speed, CurveCmd, mContext->curveSpeed);

        mContext->accelerationCmd["Curve"] = CurveCmd;
        return BT::SUCCESS;
    }

    void CurveAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
