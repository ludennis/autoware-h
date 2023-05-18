#ifndef __STATE_H__
#define __STATE_H__

#include <waypoint_follower_node_context.h>

class State
{
public:
    virtual std::shared_ptr<State> Handle(
        const WaypointFollowerNodeContext &, float & steeringCommand) = 0;
    virtual std::size_t GetClassID() const = 0;
    virtual ~State() {}
};

#endif // __STATE_H__
