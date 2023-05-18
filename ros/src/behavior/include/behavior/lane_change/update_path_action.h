#ifndef __LC_UPDATE_PATH_ACTION_H__
#define __LC_UPDATE_PATH_ACTION_H__

#include <action_node.h>
#include <behavior/context.h>
#include <behavior/lane_change/lane_change_context.h>
#include <map>
#include <memory>

namespace Behavior
{
    class UpdatePathAction : public BT::ActionNode
    {
    public:
        UpdatePathAction(const std::string & name,
            const std::shared_ptr<Context> & context,
            const std::shared_ptr<LaneChangeContext> laneChangeContext);
        BT::ReturnStatus Tick();
        void Halt();

    protected:
        void ConnectLane(const int laneId);
        BT::ReturnStatus ConstructGlobalPath();

    private:
        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<LaneChangeContext> mLaneChangeContext;
        std::vector<int> mSelectedLaneId;
        std::map<int, bool> mTravelMap;
    };
} // namespace Behavior

#endif // __LC_UPDATE_PATH_ACTION_H__
