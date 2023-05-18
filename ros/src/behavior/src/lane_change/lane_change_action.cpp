#include <behavior/lane_change/lane_change_action.h>
#include <mutex>

namespace Behavior
{
    LaneChangeAction::LaneChangeAction(
        const std::string & name,
        const std::shared_ptr<Context> & context,
        const std::shared_ptr<LaneChangeContext> laneChangeContext)
        : ActionNode::ActionNode(name)
        , mContext(context)
        , mLaneChangeContext(laneChangeContext)
    {}

    BT::ReturnStatus LaneChangeAction::Tick()
    {
        std::lock_guard<std::mutex> guard(mContext->mutex);

        std::vector<int> availableLaneId;
        for (const auto leftId : mContext->laneInfoMap[
            mLaneChangeContext->currentWaypoint.lane_id].left_lanes)
            if (!!mContext->laneInfoMap[leftId].next_lanes.size())
                availableLaneId.push_back(leftId);

        for (const auto rightId : mContext->laneInfoMap[
            mLaneChangeContext->currentWaypoint.lane_id].right_lanes)
            if (!!mContext->laneInfoMap[rightId].next_lanes.size())
                availableLaneId.push_back(rightId);

        if (!!availableLaneId.size())
            mLaneChangeContext->targetLaneId = availableLaneId.front();
        else
        {
            mLaneChangeContext->targetLaneId =
                mLaneChangeContext->currentWaypoint.lane_id;
            return BT::FAILURE;
        }

        if (is_halted())
            return BT::HALTED;
        return BT::SUCCESS;
    }

    void LaneChangeAction::Halt()
    {
        set_status(BT::HALTED);
    }
} // namespace Behavior
