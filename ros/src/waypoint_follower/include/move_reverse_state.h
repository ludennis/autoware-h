#ifndef __MOVE_REVERSE_STATE_H__
#define __MOVE_REVERSE_STATE_H__

#include <state.h>
#include <memory>
#include <type_id.h>
#include <waypoint_follower_node_context.h>
#include <signal_filter.h>
#include <controller.h>

class MoveReverseState: public State, public std::enable_shared_from_this<MoveReverseState>
{
public:
    MoveReverseState(const WaypointFollowerNodeContext &);
    std::shared_ptr<State> Handle(
        const WaypointFollowerNodeContext &, float & steeringCommand) override;
    DEFINE_TYPE_ID(MoveReverseState)

protected:
    void DoControl(
        const WaypointFollowerNodeContext &, float & steeringCmd);
    bool NeedMoveForward(const WaypointFollowerNodeContext &);

private:
    std::shared_ptr<Controller> mSteerController;
    SignalFilter mSteerCmdFilter;
};

#endif // __MOVE_REVERSE_STATE_H__
