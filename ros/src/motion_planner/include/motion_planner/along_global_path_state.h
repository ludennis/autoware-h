#ifndef __ALONG_GLOBAL_PATH_STATE_H__
#define __ALONG_GLOBAL_PATH_STATE_H__

#include <motion_planner/state.h>
#include <memory>
#include <motion_planner/type_id.h>
#include <motion_planner/motion_planner_context.h>

class AlongGlobalPathState: public State, public std::enable_shared_from_this<AlongGlobalPathState>
{
public:
    std::shared_ptr<State> Generate(
        const int indexMatch, MotionPlannerContext &) override;
    DEFINE_TYPE_ID(AlongGlobalPathState)

protected:
    std::shared_ptr<State> NextState(MotionPlannerContext &);
    void PublishLocalPaths(const MotionPlannerContext &);
    void PublishWaypoints(MotionPlannerContext &);
};

#endif // __ALONG_GLOBAL_PATH_STATE_H__
