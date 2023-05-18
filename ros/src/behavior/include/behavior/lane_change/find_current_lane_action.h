#ifndef __LC_FIND_CURRENT_LANE_ACTION_H__
#define __LC_FIND_CURRENT_LANE_ACTION_H__

#include <action_node.h>
#include <behavior/context.h>
#include <behavior/lane_change/lane_change_context.h>
#include <memory>

namespace Behavior
{
    class FindCurrentLaneAction : public BT::ActionNode
    {
    public:
        FindCurrentLaneAction(const std::string & name,
            const std::shared_ptr<Context> & context,
            const std::shared_ptr<LaneChangeContext> laneChangeContext);
        BT::ReturnStatus Tick();
        void Halt();

    private:
        BT::ReturnStatus GlobalSearching();
        route_mission_handler::Waypoint GetWaypoint(const int propertyId);

    private:
        const std::shared_ptr<Context> mContext;
        const std::shared_ptr<LaneChangeContext> mLaneChangeContext;
    };
} // namespace Behavior

#endif // __LC_FIND_CURRENT_LANE_ACTION_H__
