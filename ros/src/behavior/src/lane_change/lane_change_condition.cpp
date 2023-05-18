#include <behavior/lane_change/lane_change_condition.h>
#include <mutex>

namespace Behavior
{
    LaneChangeCondition::LaneChangeCondition(
        const std::string & name,
        const std::shared_ptr<Context> & context,
        const std::shared_ptr<LaneChangeContext> laneChangeContext)
        : ConditionNode::ConditionNode(name)
        , mContext(context)
        , mLaneChangeContext(laneChangeContext)
    {}

    BT::ReturnStatus LaneChangeCondition::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        if (!mContext->laneInfoMap[
            mLaneChangeContext->currentWaypoint.lane_id].next_lanes.size())
        {
            mLaneChangeContext->targetLaneId =
                mLaneChangeContext->currentWaypoint.lane_id;
            return BT::FAILURE;
        }
        return BT::SUCCESS;
    }

} // namespace Behavior
