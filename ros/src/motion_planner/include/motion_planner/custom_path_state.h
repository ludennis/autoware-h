#ifndef __CUSTOM_PATH_STATE_H__
#define __CUSTOM_PATH_STATE_H__

#include <motion_planner/state.h>
#include <memory>
#include <motion_planner/type_id.h>
#include <motion_planner/motion_planner_context.h>

class CustomPathState: public State, public std::enable_shared_from_this<CustomPathState>
{
public:
    std::shared_ptr<State> Generate(
        const int indexMatch, MotionPlannerContext &) override;
    DEFINE_TYPE_ID(CustomPathState)

protected:
    std::shared_ptr<State> NextState(MotionPlannerContext &);
};

#endif // __CUSTOM_PATH_STATE_H__
