#ifndef __LC_FIND_CURRENT_LANE_CONDITION_H__
#define __LC_FIND_CURRENT_LANE_CONDITION_H__

#include <behavior/context.h>
#include <behavior/lane_change/lane_change_context.h>
#include <condition_node.h>
#include <memory>

namespace Behavior
{
    class FindCurrentLaneCondition : public BT::ConditionNode
    {
    public:
        FindCurrentLaneCondition(const std::string & name,
            const std::shared_ptr<Context> & context,
            const std::shared_ptr<LaneChangeContext> laneChangeContext);
        BT::ReturnStatus Tick();

    private:
        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<LaneChangeContext> mLaneChangeContext;
    };
} // namespace Behavior

#endif // __LC_FIND_CURRENT_LANE_CONDITION_H__
