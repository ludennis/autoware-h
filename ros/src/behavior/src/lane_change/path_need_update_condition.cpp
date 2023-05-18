#include <behavior/lane_change/path_need_update_condition.h>
#include <mutex>

namespace Behavior
{
    PathNeedUpdateCondition::PathNeedUpdateCondition(
        const std::string & name,
        const std::shared_ptr<Context> & context,
        const std::shared_ptr<LaneChangeContext> laneChangeContext)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
        , mLaneChangeContext(laneChangeContext)
    {}

    BT::ReturnStatus PathNeedUpdateCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        const bool laneChangeCondition =
            mLaneChangeContext->currentWaypoint.lane_id !=
            mLaneChangeContext->targetLaneId;
        const bool InitialCondition =
            !mContext->globalPathKdtree ||
            !mContext->globalPathKdtree->getInputCloud() ||
            !mContext->globalPathKdtree->getInputCloud()->points.size();

        if (laneChangeCondition || InitialCondition)
            return BT::SUCCESS;
        else
            return BT::FAILURE;
    }
} // namespace Behavior
