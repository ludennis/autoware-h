#include <behavior/lane_change/find_current_lane_action.h>
#include <behavior/lane_change/find_current_lane_condition.h>
#include <behavior/lane_change/lane_change_action.h>
#include <behavior/lane_change/lane_change_condition.h>
#include <behavior/lane_change/lane_change_sequence.h>
#include <behavior/lane_change/path_need_update_condition.h>
#include <behavior/lane_change/update_path_action.h>
#include <parallel_node.h>

namespace Behavior
{
    LaneChangeSequence::LaneChangeSequence(
        const std::string & name,
        const std::shared_ptr<Context> & context)
        : SequenceNode::SequenceNode(name)
        , mContext(context)
        , mLaneChangeContext(std::make_shared<LaneChangeContext>())
    {
        const auto laneChangeSubSequence =
            std::make_shared<BT::SequenceNode>("laneChangeSubSequence");
        laneChangeSubSequence->AddChild(std::make_shared<LaneChangeCondition>(
            "laneChangeCondition", mContext, mLaneChangeContext));
        laneChangeSubSequence->AddChild(std::make_shared<LaneChangeAction>(
            "laneChangeAction", mContext, mLaneChangeContext));

        const auto updatePathSequence =
            std::make_shared<BT::SequenceNode>("updatePathSequence");
        updatePathSequence->AddChild(std::make_shared<PathNeedUpdateCondition>(
            "pathNeedUpdateCondition", mContext, mLaneChangeContext));
        updatePathSequence->AddChild(std::make_shared<UpdatePathAction>(
            "ppdatePathAction", mContext, mLaneChangeContext));

        const auto laneChangeParallel =
            std::make_shared<BT::ParallelNode>("laneChangeParallel", 1);
        laneChangeParallel->AddChild(laneChangeSubSequence);
        laneChangeParallel->AddChild(updatePathSequence);

        AddChild(std::make_shared<FindCurrentLaneCondition>(
            "findCurrentLaneCondition", mContext, mLaneChangeContext));
        AddChild(std::make_shared<FindCurrentLaneAction>(
            "findCurrentLaneAction", mContext, mLaneChangeContext));
        AddChild(laneChangeParallel);
    }
} // namespace Behavior
