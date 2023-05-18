#ifndef __LC_LANE_CHANGE_ACTION_H__
#define __LC_LANE_CHANGE_ACTION_H__

#include <action_node.h>
#include <behavior/context.h>
#include <behavior/lane_change/lane_change_context.h>
#include <memory>

namespace Behavior
{
    class LaneChangeAction : public BT::ActionNode
    {
    public:
        LaneChangeAction(const std::string & name,
            const std::shared_ptr<Context> & context,
            const std::shared_ptr<LaneChangeContext> laneChangeContext);
        BT::ReturnStatus Tick();
        void Halt();

    private:
        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<LaneChangeContext> mLaneChangeContext;
    };
} // namespace Behavior

#endif // __LC_LANE_CHANGE_ACTION_H__
