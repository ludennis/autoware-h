#ifndef __MOVE_FORWARD_STATE_H__
#define __MOVE_FORWARD_STATE_H__

#include <state.h>
#include <controller.h>
#include <integrator.h>
#include <memory>
#include <signal_filter.h>
#include <type_id.h>
#include <waypoint_follower_node_context.h>

class MoveForwardState: public State, public std::enable_shared_from_this<MoveForwardState>
{
public:
    MoveForwardState(const WaypointFollowerNodeContext &);
    std::shared_ptr<State> Handle(
        const WaypointFollowerNodeContext &, float & steeringCommand) override;
    DEFINE_TYPE_ID(MoveForwardState)

protected:
    void DoControl(
        const WaypointFollowerNodeContext &, float & steeringCmd);
    bool NeedMoveReverse(const WaypointFollowerNodeContext &);

private:
    std::shared_ptr<Controller> mSteerController;
    std::shared_ptr<Integrator> mCumulateLateralErr;
    SignalFilter mSteerCmdFilter;
};

#endif // __MOVE_FORWARD_STATE_H__
