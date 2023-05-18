#include <behavior/Cruise/bumper_action.h>
#include <behavior/lane_following.h>
#include <string>
#include <mutex>

static const float AVOID_SPEED = 6.0f / 3.6f; // m/s

namespace Behavior
{
    BumperAction::BumperAction(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : ActionNode::ActionNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
    {

    }

    BT::ReturnStatus BumperAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        LaneFollowing LF;

        float bumpTargetSpeed = AVOID_SPEED;

        float bumpSpeed =
            LF.ForceSpeedByDistance(
                static_cast<float>(mContext->currentBumper),
                bumpTargetSpeed,
                mContext->vehPosSD.s,
                mContext->vehicleOdometry.speed,
                mContext->speed,
                mContext->maxSpeed,
                mContext->timeStep);

        if (mContext->enterCurve) bumpSpeed = std::min(mContext->curveSpeed, bumpSpeed);
        float bumpAcceleration = (bumpSpeed - mContext->speed < 0 ?
            ((bumpSpeed - mContext->speed) / mContext->timeStep) - 1.0f :
            (bumpSpeed - mContext->speed) / mContext->timeStep);
        if (bumpAcceleration > 0.5f) bumpAcceleration = 0.5f;
        bumpSpeed = std::min(bumpTargetSpeed, bumpSpeed);

        SpeedCmd BumperCmd;
        BumperCmd.state_hint = true;
        BumperCmd.force_speed = false;
        BumperCmd.speed = AVOID_SPEED;
        BumperCmd.acceleration = -0.25f;

        mContext->accelerationCmd["Bumper"] = BumperCmd;
        return BT::SUCCESS;
    }

    void BumperAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
