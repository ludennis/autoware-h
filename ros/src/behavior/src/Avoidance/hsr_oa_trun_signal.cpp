#include <behavior/Avoidance/hsr_oa_trun_signal.h>
#include <itri_msgs/turn_signal_cmd.h>
#include <mutex>
#include <string>

static const float TURN_DISTANCE_THRESHOLD = 0.61f;

namespace Behavior
{
    hsrOATurnSignalAction::hsrOATurnSignalAction(
        std::string name,
        const std::shared_ptr<BehaviorOutput> & nodeCommand,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context,
        hsrAvoidanceSequence & nodeContext)
        : ActionNode::ActionNode(name)
        , mNodeCommand(nodeCommand)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mNodeContext(nodeContext)
    {

    }

    BT::ReturnStatus hsrOATurnSignalAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        const float diffDistance =
            mNodeContext.GetBias() - mContext->vehPosSD.d;

        if (diffDistance > TURN_DISTANCE_THRESHOLD)
            mContext->turnSignalCmd.turn_signal =
                itri_msgs::turn_signal_cmd::LEFT;
        else if (diffDistance < -TURN_DISTANCE_THRESHOLD)
            mContext->turnSignalCmd.turn_signal =
                itri_msgs::turn_signal_cmd::RIGHT;
        else
            mContext->turnSignalCmd.turn_signal =
                itri_msgs::turn_signal_cmd::NONE;

        return BT::SUCCESS;
    }


    void hsrOATurnSignalAction::Halt()
    {
        set_status(BT::HALTED);
    }
} // namespace Behavior
