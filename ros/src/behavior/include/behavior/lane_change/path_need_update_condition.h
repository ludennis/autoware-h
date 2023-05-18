#ifndef __LC_PATH_NEED_UPDATE_CONDITION_H__
#define __LC_PATH_NEED_UPDATE_CONDITION_H__

#include <behavior/context.h>
#include <behavior/lane_change/lane_change_context.h>
#include <condition_node.h>
#include <memory>

namespace Behavior
{
    class PathNeedUpdateCondition : public BT::ConditionNode
    {
    public:
        PathNeedUpdateCondition(const std::string & name,
            const std::shared_ptr<Context> & context,
            const std::shared_ptr<LaneChangeContext> laneChangeContext);
        BT::ReturnStatus Tick();

    private:
        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<LaneChangeContext> mLaneChangeContext;
    };
} // namespace Behavior

#endif // __LC_PATH_NEED_UPDATE_CONDITION_H__
