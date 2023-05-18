#include <behavior/Avoidance/hsr_reset_bias_action.h>
#include <string>
#include <mutex>
#include <visualization_msgs/Marker.h>

#define OA_DEBUG

namespace Behavior
{
    resetBiasAction::resetBiasAction(
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

    BT::ReturnStatus resetBiasAction::Tick()
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<std::mutex> guard(mContext->mutex);

        if (mNodeContext.GetBias() != 0.0f && !mNodeContext.GetCheckRightArea() && !mNodeContext.GetCheckLeftArea())
        {
            //mContext->debug->AddDebug("bias-",std::max(0.0f, mNodeContext.GetBias() - 0.04f));
            float outputbias =
                    (mNodeContext.GetBias() > 0.0f)? std::min(0.0f, mNodeContext.GetBias() - 0.1f):std::max(0.0f, mNodeContext.GetBias() + 0.1f);
            mNodeContext.SetBias(outputbias);

            mBehaviorPlanOut.hint = true;
            mBehaviorPlanOut.bias = mNodeContext.GetBias();
            mBehaviorPlanOut.update_bias = true;
            mContext->pubBehaviorPlan.publish(mBehaviorPlanOut);

#ifdef OA_DEBUG
            ROS_INFO_STREAM("The Action " << get_name() << " has succeeded");
#endif
            mContext->LaneChange = true;
            return BT::SUCCESS;
        }
        return BT::FAILURE;
    }

    void resetBiasAction::Halt()
    {
        set_status(BT::HALTED);
    }
}
