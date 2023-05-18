#include <behavior/TrafficLight/traffic_light_action.h>
#include <behavior/lane_following.h>
#include <mutex>

static const float AVOID_SPEED = 15.0f / 3.6f; // m/s

namespace Behavior
{
    TrafficLightAction::TrafficLightAction(
        std::string name,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context)
        : ActionNode::ActionNode(name)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mName(name)
    {

    }

    BT::ReturnStatus TrafficLightAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
        LaneFollowing LF;
        std::string tag;
        SpeedCmd TrafficLightCmd;

        TrafficLightCmd.state_hint = true;
        TrafficLightCmd.force_speed = true;

        float tfLightSpeed =
            LF.ForceSpeedByDistance(
                static_cast<float>(mContext->currentStopLine),
                0.0f,
                mContext->vehPosSD.s,
                mContext->vehPosSD.speed,
                mContext->speed,
                mContext->maxSpeed,
                mContext->timeStep);

        if (mContext->enterCurve)
            tfLightSpeed = std::min(mContext->curveSpeed, tfLightSpeed);

        //Concern object on gui
        if (mContext->obstacleHazard)
        {
            tag = (mContext->accSpeed < tfLightSpeed ? "ACC": mName);
        }
        else
            tag = mName;

        if (mContext->obstacleHazard)
            tfLightSpeed = std::min(mContext->accSpeed, tfLightSpeed);
        TrafficLightCmd.speed = tfLightSpeed;
        float trafficAcceleration =
            (TrafficLightCmd.speed - mContext->speed < 0 ?
            ((TrafficLightCmd.speed - mContext->speed) / mContext->timeStep) - 1.0f :
            (TrafficLightCmd.speed - mContext->speed) / mContext->timeStep);
        if (trafficAcceleration > 0.8f) trafficAcceleration = 0.8f;
        TrafficLightCmd.acceleration = trafficAcceleration;
        mContext->accelerationCmd[tag] = TrafficLightCmd;
        return BT::SUCCESS;
    }

    void TrafficLightAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
