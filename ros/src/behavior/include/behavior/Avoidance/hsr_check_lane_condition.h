#ifndef CHECK_LANE_CONDITION_H
#define CHECK_LANE_CONDITION_H

#include <condition_node.h>
#include <behavior/obstacle_avoidance.h>
#include <behavior/Avoidance/hsr_avoidance_sequence.h>

namespace Behavior
{
    class checkLaneCondition : public BT::ConditionNode
    {
    public:
        checkLaneCondition(
            std::string name,
            const std::shared_ptr<BehaviorOutput> & nodeCommand,
            const std::shared_ptr<SpeedCmd> & nodeSpeedCommand,
            const std::shared_ptr<Context> & context,
            hsrAvoidanceSequence & nodeContext);
        BT::ReturnStatus Tick();
        bool OnSideLane();

    private:
        const std::shared_ptr<BehaviorOutput> mNodeCommand;
        const std::shared_ptr<SpeedCmd> mNodeSpeedCommand;
        const std::shared_ptr<Context> mContext;
        hsrAvoidanceSequence & mNodeContext;

        float mRollInDistance;
    };
}

#endif
