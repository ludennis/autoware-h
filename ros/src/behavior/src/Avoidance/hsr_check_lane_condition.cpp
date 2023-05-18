#include <behavior/Avoidance/hsr_check_lane_condition.h>
#include <string>
#include <mutex>

// #define OA_DEBUG

namespace Behavior
{
    checkLaneCondition::checkLaneCondition(
        std::string name,
        const std::shared_ptr<BehaviorOutput> & nodeCommand,
        const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
        const std::shared_ptr<Context> & context,
        hsrAvoidanceSequence & nodeContext)
        : ConditionNode::ConditionNode(name)
        , mNodeCommand(nodeCommand)
        , mNodeSpeedCommand(nodeSpeedCommand)
        , mContext(context)
        , mNodeContext(nodeContext)
    {

    }

    BT::ReturnStatus checkLaneCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);
#ifdef OA_DEBUG
        if (OnSideLane())
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " has succeeded");
        }
        else
        {
            ROS_INFO_STREAM("The Condition " << get_name() << " is failed");
        }
#endif
        return OnSideLane() ? BT::SUCCESS : BT::FAILURE;
    }

    bool checkLaneCondition::OnSideLane()
    {
        mNodeContext.SetCheckRightArea(
            mContext->laneData[0].laneID == 0 &&
            mContext->laneData[0].laneNum != 1);
        mNodeContext.SetCheckLeftArea(
            mContext->laneData[0].laneID ==
            mContext->laneData[0].laneNum - 1 &&
            mContext->laneData[0].laneNum != 1);
        return (mNodeContext.GetCheckRightArea() || mNodeContext.GetCheckLeftArea()) &&
            mContext->trafficLight != 1;
    }
}
